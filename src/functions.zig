const std = @import("std");

const calling_convention = @import("./Assembler.zig").calling_convention;

fn sin(x: f64) callconv(calling_convention) f64 {
    return @sin(x);
}

fn cos(x: f64) callconv(calling_convention) f64 {
    return @cos(x);
}

fn tan(x: f64) callconv(calling_convention) f64 {
    return @tan(x);
}

fn asin(x: f64) callconv(calling_convention) f64 {
    return std.math.asin(x);
}

fn acos(x: f64) callconv(calling_convention) f64 {
    return std.math.acos(x);
}

fn atan(x: f64) callconv(calling_convention) f64 {
    return std.math.atan(x);
}

pub const functions = std.ComptimeStringMap(
    *const fn (x: f64) callconv(calling_convention) f64,
    .{
        .{ "sin", &sin },
        .{ "cos", &cos },
        .{ "tan", &tan },
        .{ "asin", &asin },
        .{ "acos", &acos },
        .{ "atan", &atan },
    },
);
