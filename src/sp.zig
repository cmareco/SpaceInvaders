// raylib-zig (c) Nikolas Wipper 2023

const rl = @import("raylib");
const std = @import("std");

pub fn main() anyerror!void {
    // Initialization
    //--------------------------------------------------------------------------------------
    const screenWidth = 800;
    const screenHeight = 450;

    const sound_files = [_][:0]const u8{ "resources/sounds/explosion.wav", "resources/sounds/fastinvader1.wav", "resources/sounds/fastinvader2.wav", "resources/sounds/fastinvader3.wav", "resources/sounds/fastinvader4.wav", "resources/sounds/invader_killed.wav", "resources/sounds/player_shoot.wav", "resources/sounds/ufo_highpitch.wav", "resources/sounds/ufo_lowpitch.wav" };

    var sounds: [9]rl.Sound = undefined;

    rl.initAudioDevice();
    defer rl.closeAudioDevice(); // Close audio device

    // Load all sounds
    for (sound_files, 0..) |file, i| {
        sounds[i] = rl.loadSound(file) catch blk: {
            std.debug.print("Error loading sound: {s}\n", .{file});
            break :blk undefined;
        };
    }

    rl.initWindow(screenWidth, screenHeight, "raylib-zig [core] example - basic window");
    defer rl.closeWindow(); // Close window and OpenGL context

    rl.setTargetFPS(60); // Set our game to run at 60 frames-per-second
    //--------------------------------------------------------------------------------------

    // load background image
    const image: rl.Image = rl.loadImage("resources/invaders.png") catch blk: {
        std.debug.print("Error loading background image\n", .{});
        break :blk undefined;
    };

    const background_texture = rl.loadTextureFromImage(image) catch blk: {
        std.debug.print("Error loading background texture\n", .{});
        //rl.unloadImage(image);
        break :blk undefined;
    };
    rl.unloadImage(image); // Unload image from RAM, keep texture in VRAM
    var i: u8 = 0; // Index for sound files

    // Main game loop
    while (!rl.windowShouldClose()) { // Detect window close button or ESC key
        // Update
        //----------------------------------------------------------------------------------
        // TODO: Update your variables here
        //----------------------------------------------------------------------------------

        // Draw
        //----------------------------------------------------------------------------------
        rl.beginDrawing();
        defer rl.endDrawing();
        rl.clearBackground(.white);
        if (rl.isKeyPressed(rl.KeyboardKey.p)) {
            rl.playSound(sounds[i]); // Play sound on key press
            i = (i + 1) % 8;
        }
        rl.drawTexture(background_texture, 0, 0, .white); // Draw background texture

        rl.drawText("Congrats! You created your first window!", 190, 200, 20, .light_gray);
        //----------------------------------------------------------------------------------
    }
}
