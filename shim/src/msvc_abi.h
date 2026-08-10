// MSVC C++ ABI compatibility for virtuals that return a struct in memory.
//
// THIS IS THE FIX FOR THE PHASE 0 "SteamVR dies the moment the driver loads"
// CRASH. Read this before touching GetPose() or ComputeDistortion().
//
// vrserver.exe is built with MSVC. driver_wivrnnx.dll, when cross-built with
// llvm-mingw, uses the Itanium C++ ABI. For almost every method in
// openvr_driver.h the two agree, because both follow the Win64 register
// convention. They disagree on exactly one point: a member function returning a
// class too large for a register gets a hidden pointer to the caller's return
// buffer, and the two ABIs put that pointer and `this` in opposite registers.
//
//     MSVC (vrserver):     RCX = this,        RDX = return buffer
//     Itanium (llvm-mingw): RCX = return buffer, RDX = this
//
// Verified by compiling the same source with --target=x86_64-w64-mingw32 and
// --target=x86_64-pc-windows-msvc, and by disassembling the shipped Phase 0
// DLL (HmdDevice::GetPose there does `movq %rcx,%rdi; ... memcpy(%rdi, ...)`,
// i.e. it treats RCX as the return buffer).
//
// Consequence when MSVC calls an unfixed mingw build: the driver memcpy's its
// return value straight over its own object -- the first thing it destroys is
// the vtable pointer -- and hands back the wrong pointer in RAX. The caller
// reads an uninitialised buffer and, on its next virtual call into the device,
// dereferences the smashed vptr. The access violation is raised inside
// vrserver.exe, which is why the fault module in the user's Event Viewer entry
// is vrserver.exe and not ours.
//
// The fix: define those overrides as naked functions whose entire body is a
// tail jump into an extern "C" implementation. A Win64 free function taking
// (self, out, ...) receives them in RCX, RDX, ... -- precisely the MSVC layout
// -- and returns `out` in RAX, precisely what MSVC expects back. No register
// shuffling is needed, the jump alone reinterprets the frame correctly.
//
// Two rules for anyone adding such a method later:
//   1. the class must not reach the caller through a secondary base, or the
//      compiler inserts a this-adjusting thunk that adjusts the WRONG register.
//      That is why the display component is its own single-inheritance object
//      (see display_component.h) instead of a second base of HmdDevice.
//   2. keep the extern "C" implementation's parameter list identical to the
//      virtual's, with `self` and `out` prepended.
#pragma once

#if defined(_MSC_VER)
// Building with MSVC: the compiler already emits the convention vrserver uses.
#define WNX_MSVC_SRET_FIXUP 0
#elif defined(__clang__) && defined(_WIN64)
#define WNX_MSVC_SRET_FIXUP 1
#elif defined(__GNUC__) && defined(_WIN64)
#define WNX_MSVC_SRET_FIXUP 1
#else
#define WNX_MSVC_SRET_FIXUP 0
#endif

#if WNX_MSVC_SRET_FIXUP
// Defines `ret_type Class::method()` as a bare tail jump to `impl_symbol`.
#define WNX_SRET_THUNK(ret_type, qualified_method, impl_symbol)   \
	__attribute__((naked)) ret_type qualified_method               \
	{                                                              \
		__asm__ volatile("jmp " #impl_symbol);                     \
	}
#endif
