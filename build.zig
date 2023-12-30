const std = @import("std");

pub fn build(b: *std.Build) void {
    var fw = b.addExecutable(.{
        .name = "remote.elf",
        .optimize = b.standardOptimizeOption(.{}),
        .linkage = .static,
        .root_source_file = .{ .path = "code/startup.zig" },
        .single_threaded = true,
        .target = .{
            .cpu_arch = .thumb,
            .cpu_model = .{ .explicit = &std.Target.arm.cpu.cortex_m0plus },
            .os_tag = .freestanding,
            .abi = .eabi,
        },
    });

    fw.setLinkerScriptPath(.{ .path = "linker.ld" });

    b.installArtifact(fw);
}
