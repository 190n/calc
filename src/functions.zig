const std = @import("std");

const calling_convention = @import("./Assembler.zig").calling_convention;

fn sin(x: f64) callconv(calling_convention) f64 {
    return std.math.sin(x);
}

fn sqrt(x: f64) callconv(calling_convention) f64 {
    return std.math.sqrt(x);
}

pub const functions = std.ComptimeStringMap(
    *const fn (x: f64) callconv(calling_convention) f64,
    .{
        .{ "sin", &sin },
        .{ "sqrt", &sqrt },
    },
);
