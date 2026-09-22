# Licensing and attribution

This is a repository-evidence summary, not legal advice.

## Public license

C5VRX is publicly distributed under the GNU General Public License version 3
only (`GPL-3.0-only`). The root [LICENSE](../LICENSE) contains the standard full
GPLv3 text. The original C5VRX repository was also GPL-distributed; its license
and notices remain in [`legacy/c5vrx1`](../legacy/c5vrx1/).

Copyright remains with each applicable copyright holder. A GPL grant does not
transfer copyright, and already distributed GPL versions remain available
under that grant.

Copyright holders may separately offer additional terms for portions for which
they own all necessary rights. That does not establish that the combined
repository is currently available for proprietary or dual licensing.

## Contributor evidence

The joined Git histories contain these distinct non-bot author identities:

- `Twotoz <leonbeekveldt@gmail.com>`
- `ItsReckliss <41764424+ItsReckliss@users.noreply.github.com>`
- `root <root@vmi3489225.contaboserver.net>`

GitHub also reports `ItsReckliss` as the author of a substantial historical
XIAO live-color-video contribution (archive PR #6 / commit `f9020dc3`). The
`root` identity appears on historical research-branch commits; the repository
does not establish whether that identity represents Twotoz or another person.
Authorship is not by itself definitive proof of copyright ownership, but these
records prevent a confident repository-wide claim of exclusive ownership.

Any future alternative licensing effort must review those contributions and
obtain permissions where required. No retroactive copyright assignment or CLA
is claimed.

## Dependencies, adapted material, and assets

- ESP-IDF and Espressif RF-test interfaces are external dependencies. The
  firmware includes public/private API headers but does not vendor ESP-IDF.
- The BitScrambler assembly notes reference ordering from an official Espressif
  example. Before alternative licensing, review the exact derivation and any
  attribution requirements.
- The browser flasher loads Espressif `esptool-js` 0.6.1 from a CDN; that
  dependency is not bundled into the repository.
- The C5VRX logo and branding are not granted under GPL merely because the
  software is. Historical branding terms are preserved in
  [`assets/BRANDING.md`](../assets/BRANDING.md).
- No bundled external font was identified. Other copied/adapted source or
  assets should retain their own notices and SPDX identifiers if discovered.

If commercial dual licensing becomes a concrete goal, define an explicit
future contribution-rights policy before accepting affected contributions.

