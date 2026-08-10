// The ring of side-by-side staging textures the helper encodes from
// (protocol v3, "Video frame transport" in ipc/wivrnnx_ipc.h).
//
// WHAT IT IS. Up to three full-frame textures -- both eyes flattened left/right
// into one image, so width = 2x the per-eye submitted width -- created with
// D3D11_RESOURCE_MISC_SHARED_KEYEDMUTEX | D3D11_RESOURCE_MISC_SHARED_NTHANDLE
// and exported with IDXGIResource1::CreateSharedHandle. The handle values are
// only meaningful inside vrserver; the helper duplicates them out with
// PROCESS_DUP_HANDLE (that is what ShimHello.vrserver_pid is for) and opens them
// with OpenSharedResource1.
//
// KEY DISCIPLINE (the contract's, not ours to change):
//
//     shim:   AcquireSync(0, 0ms) -> blit -> ReleaseSync(1) -> FrameReady
//     helper: AcquireSync(1)      -> encode -> ReleaseSync(0) -> FrameDone
//
// A keyed mutex hands the *next* acquirable key to whoever released it last, so
// this is self-enforcing: after our ReleaseSync(1) only an AcquireSync(1) can
// take the slot, and our own AcquireSync(0) on that slot cannot succeed again
// until the helper has released it with key 0. The zero timeout on our acquire
// turns "the helper still owns this slot" into an immediate failure instead of a
// stall in the compositor's present thread, which is exactly the backpressure
// the contract asks for: frames drop in the shim, they never queue.
//
// helper_owned is therefore belt-and-braces bookkeeping rather than the
// mechanism -- it lets us skip a slot without even asking D3D, and lets a
// FrameDone for a slot we have already recycled be recognised as stale.
//
// GENERATIONS. Every successful create() bumps the generation. FrameDone is
// matched on frame_id (which never repeats, across generations included), so a
// FrameDone that arrives for a ring we have already thrown away is ignored
// rather than freeing a slot of the new ring.
//
// THREADING. Everything is guarded by one mutex. The Present thread creates,
// destroys, leases and releases slots; the IPC thread only ever calls
// on_frame_done() and mark_stale(). Nothing here takes any other lock, so it can
// safely be called with the component's device lock held.
#pragma once

#include <wivrnnx_ipc.h>

#include <cstdint>
#include <mutex>

struct ID3D11Device;
struct ID3D11Texture2D;
struct IDXGIKeyedMutex;

namespace wivrnnx
{

class FrameStagingRing
{
public:
	// The contract caps the ring at three slots (StagingConfig::handles[3]).
	static constexpr uint32_t kMaxSlots = 3;

	// What acquire_slot() hands back: the slot index and the texture to blit
	// into. The texture pointer stays valid until the same thread releases the
	// slot, because only the Present thread ever destroys the ring.
	struct Lease
	{
		int index = -1;
		ID3D11Texture2D * texture = nullptr;

		bool valid() const noexcept
		{
			return index >= 0 && texture != nullptr;
		}
	};

	FrameStagingRing() = default;
	~FrameStagingRing();

	FrameStagingRing(const FrameStagingRing &) = delete;
	FrameStagingRing & operator=(const FrameStagingRing &) = delete;

	// --- Present thread ---------------------------------------------------

	// True when a ring exists and no disconnect has invalidated it.
	bool usable() const noexcept;
	// True when a usable ring already has exactly this geometry.
	bool matches(uint32_t width, uint32_t height, uint32_t dxgi_format) const noexcept;

	// (Re)creates the ring, bumping the generation. Any previous ring is
	// destroyed first. Returns false (and leaves no ring behind) on any D3D
	// failure; the caller degrades to dropping frames.
	bool create(ID3D11Device * device,
	            uint32_t width,
	            uint32_t height,
	            uint32_t dxgi_format,
	            uint32_t count) noexcept;
	void destroy() noexcept;

	// Fills in the StagingConfig for the current ring. False if there is none.
	bool describe(ipc::StagingConfig & out) const noexcept;

	// Takes the first slot the helper is not holding, with AcquireSync(0, 0ms).
	// An invalid Lease means "every slot is busy" -- drop the frame.
	Lease acquire_slot() noexcept;
	// Blit done: ReleaseSync(1) and remember the frame id we are about to
	// announce, so the matching FrameDone can be recognised.
	void release_to_helper(const Lease & lease, uint64_t frame_id) noexcept;
	// Blit failed: ReleaseSync(0), the slot stays ours and free.
	void return_slot(const Lease & lease) noexcept;

	uint32_t generation() const noexcept;
	uint32_t width() const noexcept;
	uint32_t height() const noexcept;
	uint32_t format() const noexcept;
	uint32_t count() const noexcept;
	// Slots currently owned by the helper, for the breadcrumbs.
	uint32_t busy_slots() const noexcept;

	// --- IPC thread -------------------------------------------------------

	// Frees the slot if the frame id still matches the lease we handed out.
	// Returns false for a stale/unknown FrameDone (already breadcrumb-worthy).
	bool on_frame_done(const ipc::FrameDone & done) noexcept;

	// The helper vanished. The keyed mutexes are left in whatever state its
	// last encode reached (possibly held, possibly abandoned), so the ring is
	// unusable; the Present thread rebuilds it from scratch on its next frame.
	// Deliberately does not touch D3D: this runs on the IPC thread while the
	// Present thread may be mid-blit.
	void mark_stale() noexcept;

private:
	struct Slot
	{
		ID3D11Texture2D * texture = nullptr;
		IDXGIKeyedMutex * mutex = nullptr;
		void * handle = nullptr; // NT handle, valid in this process
		bool helper_owned = false;
		bool leased = false; // we hold the keyed mutex with key 0
		uint64_t frame_id = 0;
	};

	void destroy_locked() noexcept;

	mutable std::mutex mutex_;
	Slot slots_[kMaxSlots]{};
	uint32_t count_ = 0;
	uint32_t width_ = 0;
	uint32_t height_ = 0;
	uint32_t format_ = 0;
	uint32_t generation_ = 0;
	bool stale_ = false;
	// Round-robin start point for acquire_slot, so a healthy ring cycles
	// through its slots instead of hammering slot 0.
	uint32_t next_ = 0;
};

} // namespace wivrnnx
