# GDB Command Reference (Practical, Human-Level)

> Scope: Commonly used **human-facing GDB commands** (covers ~95% of real debugging).
>
> VS Code integration legend:
> - ✅ = Fully via UI
> - ⚠️ = Partially / via Debug Console / unreliable
> - ❌ = Not integrated (GDB Console only, usually `-exec`)

---

## 1. Program / Execution Control

| Category | Command | Ability | VS Code |
|---|---|---|---|
| Execution | `run` | Start program | ⚠️ |
| Execution | `continue` / `c` | Resume execution | ✅ |
| Execution | `interrupt` | Pause running program | ✅ |
| Execution | `step` / `s` | Step into | ✅ |
| Execution | `next` / `n` | Step over | ✅ |
| Execution | `finish` | Run until function returns | ⚠️ |
| Execution | `until` | Run until line / out of scope | ❌ |
| Execution | `jump` | Change PC to location | ❌ |
| Execution | `kill` | Kill debuggee | ⚠️ |
| Execution | `detach` | Detach debugger | ⚠️ |

---

## 2. Breakpoints

| Category | Command | Ability | VS Code |
|---|---|---|---|
| Breakpoint | `break` / `b` | Set breakpoint | ✅ |
| Breakpoint | `tbreak` | One-shot breakpoint | ❌ |
| Breakpoint | `delete` | Remove breakpoint | ✅ |
| Breakpoint | `disable` | Disable breakpoint | ✅ |
| Breakpoint | `enable` | Enable breakpoint | ✅ |
| Breakpoint | `info breakpoints` | List breakpoints | ⚠️ |
| Breakpoint | `condition` | Add condition | ⚠️ |
| Breakpoint | `ignore` | Ignore first N hits | ❌ |
| Breakpoint | `commands` | Attach commands to breakpoint | ❌ |

---

## 3. Watchpoints

| Category | Command | Ability | VS Code |
|---|---|---|---|
| Watchpoint | `watch` | Break on write | ⚠️ |
| Watchpoint | `rwatch` | Break on read | ❌ |
| Watchpoint | `awatch` | Break on read/write | ❌ |
| Watchpoint | `info watchpoints` | List watchpoints | ❌ |

---

## 4. Stack / Call Frames

| Category | Command | Ability | VS Code |
|---|---|---|---|
| Stack | `bt` | Backtrace | ✅ |
| Stack | `bt full` | Backtrace + locals | ❌ |
| Stack | `frame` | Select frame | ⚠️ |
| Stack | `up` / `down` | Navigate frames | ⚠️ |
| Stack | `info frame` | Frame details | ❌ |
| Stack | `info locals` | Local variables | ⚠️ |
| Stack | `info args` | Function arguments | ⚠️ |

---

## 5. Threads / RTOS

| Category | Command | Ability | VS Code |
|---|---|---|---|
| Threads | `info threads` | List threads | ⚠️ |
| Threads | `thread` | Switch thread | ⚠️ |
| Threads | `thread apply all bt` | All-thread backtrace | ❌ |
| Threads | `set scheduler-locking on` | Freeze other threads | ❌ |

---

## 6. Variables & Expressions

| Category | Command | Ability | VS Code |
|---|---|---|---|
| Variables | `print` / `p` | Evaluate expression | ✅ |
| Variables | `display` | Auto-print on stop | ❌ |
| Variables | `set variable` | Modify variable | ⚠️ |
| Variables | `call` | Call function | ❌ |
| Variables | `ptype` | Show type | ❌ |
| Variables | `whatis` | Show type name | ❌ |

---

## 7. Memory Inspection

| Category | Command | Ability | VS Code |
|---|---|---|---|
| Memory | `x` | Examine memory | ❌ |
| Memory | `set {type}addr = val` | Write memory | ❌ |
| Memory | `info proc mappings` | Memory layout (Linux) | ❌ |

---

## 8. Registers / CPU State

| Category | Command | Ability | VS Code |
|---|---|---|---|
| Registers | `info registers` | Dump registers | ❌ |
| Registers | `p $pc / $sp / $lr` | Read registers | ❌ |
| Registers | `set $pc = …` | Modify register | ❌ |

---

## 9. Assembly / Low-Level

| Category | Command | Ability | VS Code |
|---|---|---|---|
| Assembly | `disassemble` | Disassemble function | ❌ |
| Assembly | `x/i` | Disassemble at address | ❌ |
| Assembly | `stepi` | Step instruction | ❌ |
| Assembly | `nexti` | Step over instruction | ❌ |
| Assembly | `layout asm` | ASM TUI view | ❌ |

---

## 10. Faults / Signals / Exceptions

| Category | Command | Ability | VS Code |
|---|---|---|---|
| Fault | `info signals` | Signal list | ❌ |
| Fault | `handle` | Control signal behavior | ❌ |
| Fault | `catch throw` | Catch C++ exceptions | ❌ |
| Fault | `catch syscall` | Catch syscalls | ❌ |

---

## 11. Remote / Embedded-Specific

| Category | Command | Ability | VS Code |
|---|---|---|---|
| Remote | `target remote` | Attach to target | ⚠️ |
| Remote | `monitor` | Send command to GDB server | ❌ |
| Remote | `load` | Flash program | ⚠️ |
| Remote | `compare-sections` | Verify flash | ❌ |
| Remote | `set architecture` | Set target architecture | ❌ |
| Remote | `set endian` | Set endianness | ❌ |

---

## 12. Scripting / Automation

| Category | Command | Ability | VS Code |
|---|---|---|---|
| Scripting | `define` | Define custom command | ❌ |
| Scripting | `document` | Help for custom command | ❌ |
| Scripting | `.gdbinit` | Startup script | ❌ |
| Scripting | `source` | Load script | ❌ |
| Scripting | `python` | Python scripting | ❌ |

---

## Notes

- VS Code integrates roughly **25–30%** of GDB functionality.
- The remaining **70%** is essential for embedded, RTOS, fault, and memory debugging.
- For Cortex-M / Zephyr / STM32 work, expect to rely heavily on **GDB Console** commands.

