const std = @import("std");
const rlz = @import("raylib_zig");

pub fn build(b: *std.Build) !void {
    const target = b.standardTargetOptions(.{});
    const optimize = b.standardOptimizeOption(.{});

    const raylib_dep = b.dependency("raylib_zig", .{
        .target = target,
        .optimize = optimize,
    });

    const raylib = raylib_dep.module("raylib");
    const raylib_artifact = raylib_dep.artifact("raylib");

    //web exports are completely separate
    if (target.query.os_tag == .emscripten) {
        const exe_lib = try rlz.emcc.compileForEmscripten(b, "i8080", "src/main.zig", target, optimize);

        exe_lib.linkLibrary(raylib_artifact);
        exe_lib.root_module.addImport("raylib", raylib);

        // Note that raylib itself is not actually added to the exe_lib output file, so it also needs to be linked with emscripten.
        const link_step = try rlz.emcc.linkWithEmscripten(b, &[_]*std.Build.Step.Compile{ exe_lib, raylib_artifact });
        //this lets your program access files like "resources/my-image.png":
        link_step.addArg("--embed-file");
        link_step.addArg("resources/");

        b.getInstallStep().dependOn(&link_step.step);
        const run_step = try rlz.emcc.emscriptenRunStep(b);
        run_step.step.dependOn(&link_step.step);
        const run_option = b.step("run", "Run i8080");
        run_option.dependOn(&run_step.step);
        return;
    }

    const raylib_test = b.addTest(.{
        .root_source_file = b.path("lib/raylib.zig"),
        .target = target,
        .optimize = optimize,
    });
    raylib_test.linkLibC();

    // original exe with raylib
    // const exe = b.addExecutable(.{ .name = "i8080", .root_source_file = b.path("src/8080.zig"), .optimize = optimize, .target = target });
    // exe.linkLibrary(raylib_artifact);
    // exe.root_module.addImport("raylib", raylib);
    // const run_cmd = b.addRunArtifact(exe);
    // const run_step = b.step("run", "Run i8080");
    // run_step.dependOn(&run_cmd.step);
    // b.installArtifact(exe);

    // 8080 executable
    const exe = b.addExecutable(.{ .name = "i8080", .root_source_file = b.path("src/8080.zig"), .optimize = optimize, .target = target });
    const run_cmd = b.addRunArtifact(exe);
    const run_step = b.step("run", "Run i8080");
    run_step.dependOn(&run_cmd.step);
    b.installArtifact(exe);

    // exercise 8080 executable
    const exercise = b.addExecutable(.{ .name = "exercise_8080", .root_source_file = b.path("src/exercise_8080.zig"), .optimize = optimize, .target = target });
    const exercise_step = b.step("exercise", "Build exercise");
    const install_exercise = b.addInstallArtifact(exercise, .{});
    exercise_step.dependOn(&install_exercise.step);

    // space invaders executable
    const si_exe = b.addExecutable(.{ .name = "SpaceInvaders", .root_source_file = b.path("src/space_invaders.zig"), .optimize = optimize, .target = target });
    si_exe.linkLibrary(raylib_artifact);
    si_exe.root_module.addImport("raylib", raylib);
    const si_run_cmd = b.addRunArtifact(si_exe);
    const si_run_step = b.step("space", "Run SpaceInvaders");
    si_run_step.dependOn(&si_run_cmd.step);
    const install_si = b.addInstallArtifact(si_exe, .{});
    si_run_step.dependOn(&install_si.step);
    b.installArtifact(si_exe);

    // space invaders executable
    const sp_exe = b.addExecutable(.{ .name = "sp", .root_source_file = b.path("src/sp.zig"), .optimize = optimize, .target = target });
    sp_exe.linkLibrary(raylib_artifact);
    sp_exe.root_module.addImport("raylib", raylib);
    const sp_run_cmd = b.addRunArtifact(sp_exe);
    const sp_run_step = b.step("sp", "Run SP");
    sp_run_step.dependOn(&sp_run_cmd.step);
    const install_sp = b.addInstallArtifact(sp_exe, .{});
    sp_run_step.dependOn(&install_sp.step);
    b.installArtifact(sp_exe);

    //     // tests
    //     const exe_tests = b.addTest(.{ .root_source_file = b.path("src/space_invaders.zig"), .target = target, .optimize = optimize });
    //     exe_tests.linkLibrary(raylib_artifact);
    //     exe_tests.root_module.addImport("raylib", raylib);
    //     const test_step = b.step("test", "Run unit tests");
    //     test_step.dependOn(&exe_tests.step);
    //
}
