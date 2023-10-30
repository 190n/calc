const std = @import("std");

const Program = @import("./program.zig");
const AsmBuf = @import("./asmbuf.zig").AsmBuf;

const CompiledCode = *const fn (stack: [*]f64, constants: [*]const f64) callconv(.C) void;

const Diagnostic = union(enum) {
    none: void,
    actual_num_args: usize,
    invalid_numeric_literal: []const u8,
};

fn execLine(
    code: CompiledCode,
    program: Program,
    line: []const u8,
    stdout: std.fs.File.Writer,
    constants: []const f64,
    diagnostic: ?*Diagnostic,
) !void {
    var it = std.mem.tokenizeScalar(u8, line, ' ');
    var index: usize = 0;
    var stack: [257]f64 = undefined;
    while (it.next()) |substring| {
        const arg = std.fmt.parseFloat(f64, substring) catch |e| {
            if (diagnostic) |d| d.* = .{ .invalid_numeric_literal = substring };
            return e;
        };
        stack[index] = arg;
        index += 1;
    }

    if (index != program.num_args) {
        if (diagnostic) |d| d.* = .{ .actual_num_args = index };
        return error.WrongNumberOfArguments;
    }

    code(&stack, constants.ptr);

    if (program.num_returns == 0) {
        try stdout.writeAll("(empty stack)\n");
    } else {
        try stdout.print("{d}", .{stack[0]});
        for (stack[1..program.num_returns]) |f| {
            try stdout.print("{d} ", .{f});
        }
        try stdout.print("\n", .{});
    }
}

fn run(argv: [][:0]u8, stdout: std.fs.File.Writer, stderr: std.fs.File.Writer) !void {
    var buf = try AsmBuf.create();
    defer buf.destroy();

    if (argv.len != 2) {
        return error.WrongNumberOfArguments;
    }

    const program = try Program.parse(argv[1]);
    const constants = try program.compile(buf);
    const func = try buf.finalize(CompiledCode);

    var input = std.io.getStdIn().reader();
    var line_buf: [1024]u8 = undefined;

    try stderr.writeAll("> ");

    while (try input.readUntilDelimiterOrEof(&line_buf, '\n')) |line| {
        var diagnostic = Diagnostic{ .none = {} };

        execLine(func, program, line, stdout, &constants, &diagnostic) catch |e| {
            std.log.err("{s}", .{@errorName(e)});
            switch (e) {
                error.WrongNumberOfArguments => std.log.info(
                    "the supplied program requires {} arguments, but {} were provided",
                    .{ program.num_args, diagnostic.actual_num_args },
                ),
                error.InvalidCharacter => std.log.info(
                    "\"{s}\" is not a valid number",
                    .{diagnostic.invalid_numeric_literal},
                ),
                else => continue,
            }
        };

        try stderr.writeAll("> ");
    }
}

const usage =
    \\usage: {s} <program>
    \\    program is a RPN expression which starts with some arguments (user input) already on the
    \\    stack. supported: decimal constants, +, -, *, /. if program is '+ 2 *', then after running
    \\    the user enters two numbers, and twice their sum will be printed.
;

pub fn main() !void {
    var gpa = std.heap.GeneralPurposeAllocator(.{}){};
    defer std.debug.assert(gpa.deinit() == .ok);
    const allocator = gpa.allocator();
    const argv = try std.process.argsAlloc(allocator);
    defer std.process.argsFree(allocator, argv);

    const stdout = std.io.getStdOut().writer();
    const stderr = std.io.getStdErr().writer();

    run(argv, stdout, stderr) catch |e| switch (e) {
        else => return e,
    };
}
