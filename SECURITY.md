# Security Policy

## Reporting

Report privately through GitHub: **Security → Report a vulnerability**.
Please do not open a public issue for anything you believe is exploitable.

Single maintainer, spare time. Expect a first reply within about a week as a
best effort, not a commitment. No bounty.

## Scope

These components have no network code, no parsing, no allocation and no
dependencies. They do arithmetic on numbers the caller supplies, in storage the
caller owns. What is left is memory safety in C — and that matters, because
this code is meant to be copied into other people's firmware.

**In scope:** out-of-bounds reads or writes, uninitialised reads, undefined
behaviour, unbounded stack growth, or a validation function accepting a config
that later causes any of those.

**Out of scope:** bad tuning producing bad numbers, numerical inaccuracy, and
anything in `examples/`. Those are ordinary issues, not vulnerabilities.
