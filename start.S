.sect ".init0"
.global _start

_start:
    ldi32 sp, _stack_top
    jmp main
