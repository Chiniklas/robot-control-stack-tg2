# RCS TG2 Extension (scaffold)

This is a scaffolded extension mirroring the layout of `rcs_fr3`, intended for a future TG2 hardware integration.
It currently provides Python stubs only (no hardware bindings) so you can begin wiring environments and CLI commands
without compiled code. Replace the stub implementations with real drivers when available.

## Installation (dev)
```shell
pip install -ve .
```

## Status
- Hardware bindings: not implemented (Python stubs only)
- CLI: placeholder commands
- Env creators: structure matches `rcs_fr3` but use stubs

## Next steps
- Fill `_core/hw.py` with real device control
- Update default configs in `utils.py` once TG2 scene/URDF is available
- Extend the Typer CLI in `__main__.py` with real maintenance commands
```
