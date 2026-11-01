"""Training tasks, environments, and MDP definitions for Berkeley Humanoid Lite."""


def __getattr__(name: str):
    if name == "configs":
        import importlib
        return importlib.import_module("trainning.configs")
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
