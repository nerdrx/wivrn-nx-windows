// The IVRDisplayComponent SteamVR fetches from the HMD via GetComponent.
//
// Deliberately a standalone object with a single base rather than a second base
// of HmdDevice. ComputeDistortion returns a struct in memory and therefore has
// to be emitted with the MSVC this/sret register order (see msvc_abi.h); a
// method reached through a secondary base also gets a compiler-generated
// this-adjusting thunk, and that thunk would adjust whichever register the
// compiler believes holds `this` -- the wrong one. With single inheritance no
// thunk is generated and our naked override is the only entry point.
#pragma once

#include "openvr_driver_wrap.h"

namespace wivrnnx
{

class HmdDevice;

class DisplayComponent final : public vr::IVRDisplayComponent
{
public:
	explicit DisplayComponent(HmdDevice * owner) noexcept :
	        owner_(owner) {}

	HmdDevice * owner() const noexcept
	{
		return owner_;
	}

	void GetWindowBounds(int32_t * x, int32_t * y, uint32_t * width, uint32_t * height) override;
	bool IsDisplayOnDesktop() override;
	bool IsDisplayRealDisplay() override;
	void GetRecommendedRenderTargetSize(uint32_t * width, uint32_t * height) override;
	void GetEyeOutputViewport(vr::EVREye eye, uint32_t * x, uint32_t * y, uint32_t * width, uint32_t * height) override;
	void GetProjectionRaw(vr::EVREye eye, float * left, float * right, float * top, float * bottom) override;
	vr::DistortionCoordinates_t ComputeDistortion(vr::EVREye eye, float u, float v) override;
	bool ComputeInverseDistortion(vr::HmdVector2_t * result, vr::EVREye eye, uint32_t channel, float u, float v) override;

private:
	HmdDevice * owner_ = nullptr;
};

} // namespace wivrnnx
