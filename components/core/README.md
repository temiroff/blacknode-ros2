# Core

Component of `blacknode-ros2`.

Node sources for this component belong in this folder. Until they move here,
nodes claim the component inline:

    @node(name="MyNode", component="core", ...)

Once sources live here, declare the folder in `blacknode-package.toml`:

    [components.core]
    nodes = ["components/core/nodes"]

and the inline `component=` argument can be dropped — the loader infers it
from the directory.
