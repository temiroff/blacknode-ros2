# Actions

Component of `blacknode-ros2`.

Node sources for this component belong in this folder. Until they move here,
nodes claim the component inline:

    @node(name="MyNode", component="actions", ...)

Once sources live here, declare the folder in `blacknode-package.toml`:

    [components.actions]
    nodes = ["components/actions/nodes"]

and the inline `component=` argument can be dropped — the loader infers it
from the directory.
