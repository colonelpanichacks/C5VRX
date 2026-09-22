# Archive issue #24 — Hardware dataplane exploration

| Field | Value |
|---|---|
| Repository | `Twotoz/C5VRX-archive` |
| Created | 2026-08-27 |
| Closed | 2026-08-29 |
| Original | [Issue #24](https://github.com/Twotoz/C5VRX-archive/issues/24) |

## Historical hypothesis

The issue explored moving the hot path into GDMA, BitScrambler, PARLIO, and
small line-oriented buffers. It collected useful peripheral possibilities but
mixed several unproven architectures into one checklist.

## Historical conclusion

The issue was closed as too broad and speculative. Its closing guidance says
not to treat it as an implementation checklist.

## Later status

Some primitives were later validated independently—especially PARLIO RX/TX
and TX-attached BitScrambler rate conversion—but the modern topology must be
read from current source and `/docs`, not reconstructed from this issue.

