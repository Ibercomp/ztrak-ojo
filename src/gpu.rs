//! Windows: all drawing through DirectX 12, the same path as ZTrak on the Arc A310.
//! Mixing OpenGL, Vulkan and OpenVINO on that card gave NaN outputs and device-lost resets.
//! `WGPU_BACKEND` (vulkan, dx12...) still overrides it for tests.
use eframe::egui_wgpu::{WgpuConfiguration, WgpuSetup, wgpu};

fn instance_descriptor() -> wgpu::InstanceDescriptor {
    if std::env::var_os("WGPU_BACKEND").is_some() {
        return wgpu::InstanceDescriptor::new_without_display_handle_from_env();
    }
    let mut d = wgpu::InstanceDescriptor::new_without_display_handle();
    d.backends = wgpu::Backends::DX12;
    let dxc = std::env::current_exe()
        .ok()
        .and_then(|e| e.parent().map(|p| p.join("dxcompiler.dll")))
        .filter(|p| p.exists());
    d.backend_options.dx12.shader_compiler = match dxc {
        Some(p) => wgpu::Dx12Compiler::DynamicDxc { dxc_path: p.display().to_string() },
        None => {
            log::warn!("Falta dxcompiler.dll junto al programa: DirectX 12 con el compilador antiguo");
            wgpu::Dx12Compiler::Auto
        }
    };
    d
}

/// Normal vsync and at most two queued images: the A310 has 4 GB and shares them with ZTrak.
pub fn eframe_wgpu() -> WgpuConfiguration {
    let mut c = WgpuConfiguration::default();
    c.surface.present_mode = wgpu::PresentMode::AutoVsync;
    c.surface.desired_maximum_frame_latency = Some(2);
    if let WgpuSetup::CreateNew(s) = &mut c.wgpu_setup {
        s.instance_descriptor = instance_descriptor();
        s.power_preference = wgpu::PowerPreference::HighPerformance;
    }
    c
}
