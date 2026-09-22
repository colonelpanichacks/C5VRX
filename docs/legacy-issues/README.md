# Legacy GitHub research

This directory preserves the technically useful conclusions from every GitHub
issue in `Twotoz/C5VRX-archive`. Git commits do not preserve issue bodies or
comments, so these notes are part of the project migration.

The notes deliberately distinguish historical hypotheses from measurements.
They are summaries, not replacements for the original discussions while the
archive repository remains online. Current hardware evidence in `/docs` takes
precedence whenever it conflicts with an older conclusion.

| Issue | Topic | Historical disposition |
|---:|---|---|
| [#5](issue-05.md) | Production AV path | Still useful as lineage; implementation superseded |
| [#7](issue-07.md) | Repository/configurator roadmap | Partially relevant; old migration gate superseded |
| [#19](issue-19.md) | Continuous RF-to-AV pacing | Closed and superseded |
| [#20](issue-20.md) | Startup underrun/watchdog | Confirmed historical failure |
| [#22](issue-22.md) | Finite 16K restart boundary | Measured, but no gapless proof; superseded path |
| [#24](issue-24.md) | Hardware dataplane exploration | Closed as overly broad/speculative |
| [#27](issue-27.md) | Adjacent FM and direct CVBS | Core principles confirmed; topology evolved |
| [#28](issue-28.md) | Native continuous IQ architecture | Major direction; partially confirmed and revised |

High-signal pull-request discussions are summarized in
[pull-requests.md](pull-requests.md).

