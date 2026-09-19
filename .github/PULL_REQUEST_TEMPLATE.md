<!-- Please read CONTRIBUTING.md first. It is short, and the invariants in it
     protect real hardware. -->

## What changed, and why
<!-- The failure or need that motivated this, not just the diff. -->

## How it was checked
<!-- What you actually ran. For a hardware claim, give the evidence: which board,
     the topic rates, the boot console, the flash log. "Should work" is not a
     test result. Say what you did NOT run. -->

- [ ] `python3 -m pytest tests`
- [ ] Static checks (`py_compile`, `node --check`, the migration dry run)
- [ ] Firmware built for the affected boards, if firmware changed
- [ ] Documentation updated next to the rule it changes
- [ ] Nothing private: no hostnames, addresses, Wi-Fi names or credentials
