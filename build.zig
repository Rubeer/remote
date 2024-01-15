const std = @import("std");

pub fn build(b: *std.Build) void {
    const target = b.resolveTargetQuery(.{
        .cpu_arch = .thumb,
        .cpu_model = .{ .explicit = &std.Target.arm.cpu.cortex_m0plus },
        .os_tag = .freestanding,
        .abi = .eabi,
    });

    var fw = b.addExecutable(.{
        .name = "remote.elf",
        .optimize = b.standardOptimizeOption(.{}),
        .linkage = .static,
        .root_source_file = .{ .path = "code/startup.zig" },
        .single_threaded = true,
        .target = target,
        .strip = false,
    });

    fw.entry = .{ .symbol_name = "Reset_Handler" };
    fw.link_function_sections = true;
    fw.link_data_sections = true;
    fw.link_gc_sections = true;
    fw.setLinkerScriptPath(.{ .path = "linker.ld" });

    b.installArtifact(fw);
}
