const std = @import("std");
const builtin = @import("builtin");

const AsmBuf = @import("./asmbuf.zig").AsmBuf;
const Compiler = @import("./compiler.zig");

const Program = @This();

pub const Op = union(enum) {
    add: void,
    sub: void,
    mul: void,
    div: void,
    constant: f64,
};

code: std.BoundedArray(Op, 256) = .{},
peak_stack_size: u16 = 0,
num_args: u16 = 0,
num_returns: u16 = 0,

pub const ParseError = std.fmt.ParseFloatError || error{ InvalidProgram, TooLong };

pub fn parse(texts: [][]const u8, erroneous_part: ?*[]const u8) ParseError!Program {
    var p = Program{};
    var current_stack_size: u16 = 0;

    for (texts) |text| {
        var it = std.mem.tokenizeScalar(u8, text, ' ');
        while (it.next()) |s| {
            if (std.fmt.parseFloat(f64, s)) |f| {
                p.code.append(.{ .constant = f }) catch return error.TooLong;
                current_stack_size += 1;
                p.peak_stack_size = @max(current_stack_size, p.peak_stack_size);
            } else |_| {
                if (s.len == 1) {
                    p.code.append(switch (s[0]) {
                        '+' => .add,
                        '-' => .sub,
                        '*' => .mul,
                        '/' => .div,
                        else => {
                            if (erroneous_part) |e| e.* = s;
                            return error.InvalidProgram;
                        },
                    }) catch return error.TooLong;

                    while (current_stack_size < 2) {
                        current_stack_size += 1;
                        p.peak_stack_size += 1;
                        p.num_args += 1;
                    }

                    current_stack_size -= 1;
                } else {
                    if (erroneous_part) |e| e.* = s;
                    return error.InvalidProgram;
                }
            }
        }
    }

    p.num_returns = current_stack_size;
    return p;
}

pub fn compile(self: Program, asm_buf: *AsmBuf) ![256]f64 {
    var compiler = Compiler.initWithConstants(self);

    for (self.code.slice()) |op| {
        try compiler.compileOperator(asm_buf, op);
    }
    try compiler.addReturn(asm_buf);
    return compiler.constants;
}
