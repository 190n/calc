const std = @import("std");
const builtin = @import("builtin");

const Program = @import("./Program.zig");
const AsmBuf = @import("./AsmBuf.zig").AsmBuf;
const Compiler = @import("./Compiler.zig");

const ExecDiagnostic = union {
    none: void,
    actual_num_args: usize,
    invalid_numeric_literal: []const u8,
};

fn execLine(
    code: Compiler.CompiledCode,
    program: Program,
    line: []const u8,
    stdout: std.fs.File.Writer,
    constants: []const f64,
    diagnostic: ?*ExecDiagnostic,
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
            try stdout.print(" {d}", .{f});
        }
        try stdout.print("\n", .{});
    }
}

fn run(
    argv: [][:0]u8,
    stdout: std.fs.File.Writer,
    stderr: std.fs.File.Writer,
    erroneous_part: ?*[]const u8,
    allocator: std.mem.Allocator,
) !void {
    var buf = try AsmBuf.create();
    defer buf.destroy();

    if (argv.len < 2) {
        return error.WrongNumberOfArguments;
    }

    var program = try Program.parse(allocator, argv[1..], erroneous_part);
    defer program.deinit();
    const constants = try program.compile(buf);
    defer allocator.free(constants);
    const func = try buf.finalize(Compiler.CompiledCode);

    var input = std.io.getStdIn().reader();
    var line_buf: [1024]u8 = undefined;

    try stderr.writeAll("> ");

    while (try input.readUntilDelimiterOrEof(&line_buf, '\n')) |raw_line| {
        const line = std.mem.trimRight(u8, raw_line, "\r");

        var diagnostic = ExecDiagnostic{ .none = {} };
        execLine(func, program, line, stdout, constants, &diagnostic) catch |e| {
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
    \\
;

pub fn main() !void {
    var arena = std.heap.ArenaAllocator.init(std.heap.page_allocator);

    var gpa = std.heap.GeneralPurposeAllocator(.{}){};

    const allocator = switch (builtin.mode) {
        .Debug => gpa.allocator(),
        else => arena.allocator(),
    };
    defer switch (builtin.mode) {
        .Debug => std.debug.assert(gpa.deinit() == .ok),
        else => arena.deinit(),
    };

    const argv = try std.process.argsAlloc(allocator);
    defer std.process.argsFree(allocator, argv);

    const stdout = std.io.getStdOut().writer();
    const stderr = std.io.getStdErr().writer();

    for (argv[1..]) |s| {
        if (std.mem.eql(u8, s, "-h")) {
            try stderr.print(usage, .{argv[0]});
            return;
        }
    }

    var erroneous_part: []const u8 = &.{};

    run(argv, stdout, stderr, &erroneous_part, allocator) catch |e| {
        std.log.err("{s}", .{@errorName(e)});
        switch (e) {
            error.InvalidProgram => std.log.info("\"{s}\" is not a valid number or operator", .{erroneous_part}),
            error.TooLong => std.log.info("program has too many instructions", .{}),
            error.WrongNumberOfArguments => try stderr.print(usage, .{argv[0]}),
            else => {},
        }
        std.log.info("use \"{s} -h\" for usage", .{argv[0]});
        std.process.exit(1);
    };
}
