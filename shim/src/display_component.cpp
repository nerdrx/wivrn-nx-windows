#include "display_component.h"

#include "breadcrumb.h"
#include "hmd_device.h"
#include "msvc_abi.h"

namespace wivrnnx
{
namespace
{

ipc::HmdConfig config_of(const HmdDevice * owner) noexcept
{
	if (owner != nullptr)
		return owner->config_copy();

	// Should never happen, but a display component with no owner still has to
	// answer plausibly rather than divide by zero somewhere in the compositor.
	ipc::HmdConfig fallback{};
	fallback.eye_width = 1920;
	fallback.eye_height = 1920;
	fallback.refresh_hz = 90.0f;
	fallback.ipd_m = 0.063f;
	for (int eye = 0; eye < 2; ++eye)
	{
		fallback.proj_left[eye] = -1.0f;
		fallback.proj_right[eye] = 1.0f;
		fallback.proj_top[eye] = -1.0f;
		fallback.proj_bottom[eye] = 1.0f;
	}
	return fallback;
}

} // namespace

void DisplayComponent::GetWindowBounds(int32_t * x, int32_t * y, uint32_t * width, uint32_t * height)
{
	try
	{
		const ipc::HmdConfig config = config_of(owner_);
		const uint32_t total_width = config.eye_width * 2;

		// Fictional extended display: park it immediately to the left of the
		// primary desktop so it can never overlap a real monitor.
		if (x != nullptr)
			*x = -static_cast<int32_t>(total_width);
		if (y != nullptr)
			*y = 0;
		if (width != nullptr)
			*width = total_width;
		if (height != nullptr)
			*height = config.eye_height;
	}
	catch (...)
	{
	}
}

bool DisplayComponent::IsDisplayOnDesktop()
{
	return false;
}

bool DisplayComponent::IsDisplayRealDisplay()
{
	return false;
}

void DisplayComponent::GetRecommendedRenderTargetSize(uint32_t * width, uint32_t * height)
{
	try
	{
		const ipc::HmdConfig config = config_of(owner_);
		if (width != nullptr)
			*width = config.eye_width;
		if (height != nullptr)
			*height = config.eye_height;
	}
	catch (...)
	{
	}
}

void DisplayComponent::GetEyeOutputViewport(vr::EVREye eye, uint32_t * x, uint32_t * y, uint32_t * width, uint32_t * height)
{
	try
	{
		const ipc::HmdConfig config = config_of(owner_);
		if (x != nullptr)
			*x = (eye == vr::Eye_Left) ? 0 : config.eye_width;
		if (y != nullptr)
			*y = 0;
		if (width != nullptr)
			*width = config.eye_width;
		if (height != nullptr)
			*height = config.eye_height;
	}
	catch (...)
	{
	}
}

void DisplayComponent::GetProjectionRaw(vr::EVREye eye, float * left, float * right, float * top, float * bottom)
{
	try
	{
		const ipc::HmdConfig config = config_of(owner_);
		const int index = (eye == vr::Eye_Left) ? 0 : 1;
		if (left != nullptr)
			*left = config.proj_left[index];
		if (right != nullptr)
			*right = config.proj_right[index];
		if (top != nullptr)
			*top = config.proj_top[index];
		if (bottom != nullptr)
			*bottom = config.proj_bottom[index];
	}
	catch (...)
	{
	}
}

bool DisplayComponent::ComputeInverseDistortion(vr::HmdVector2_t * result, vr::EVREye, uint32_t, float u, float v)
{
	if (result == nullptr)
		return false;
	result->v[0] = u;
	result->v[1] = v;
	return true;
}

// --- ComputeDistortion: struct return, needs the MSVC register order --------

extern "C" vr::DistortionCoordinates_t * wnx_display_compute_distortion(DisplayComponent * self,
                                                                        vr::DistortionCoordinates_t * out,
                                                                        vr::EVREye eye,
                                                                        float u,
                                                                        float v) noexcept
{
	(void) self;
	(void) eye;
	if (out == nullptr)
		return out;

	// The client headset applies its own distortion; ours is the identity.
	out->rfRed[0] = u;
	out->rfRed[1] = v;
	out->rfGreen[0] = u;
	out->rfGreen[1] = v;
	out->rfBlue[0] = u;
	out->rfBlue[1] = v;
	return out;
}

#if WNX_MSVC_SRET_FIXUP
WNX_SRET_THUNK(vr::DistortionCoordinates_t,
               DisplayComponent::ComputeDistortion(vr::EVREye, float, float),
               wnx_display_compute_distortion)
#else
vr::DistortionCoordinates_t DisplayComponent::ComputeDistortion(vr::EVREye eye, float u, float v)
{
	vr::DistortionCoordinates_t coordinates{};
	wnx_display_compute_distortion(this, &coordinates, eye, u, v);
	return coordinates;
}
#endif

} // namespace wivrnnx
