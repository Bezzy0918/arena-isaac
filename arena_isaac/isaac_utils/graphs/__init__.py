
import typing

import omni.graph.core as og


class _Node:
    def __init__(
        self,
        master: "Graph",
        name: str,
        type_: str,
        *,
        values: typing.Iterable[tuple[str, typing.Any]] | None = None,
        connections: typing.Iterable[tuple[str, "_Node", str]] | None = None,
    ) -> None:
        self._master: "Graph" = master
        self._name: str = name
        self._type: str = type_

        if values is not None:
            for value in values:
                self.attribute(*value)

        if connections is not None:
            for connection in connections:
                self.connect(*connection)

    @property
    def name(self) -> str:
        return self._name

    @property
    def path(self) -> str:
        return f"{self._master.path}/{self._name}"

    @property
    def type(self) -> str:
        return self._type

    def create_attribute(self, attribute: str, type_: str):
        self._master.add_action(
            lambda: og.Controller.create_attribute(
                self.path,
                attribute,
                type_
            )
        )

    def attribute(self, input_: str, value: typing.Any):
        self._master.add_action(
            lambda: og.Controller.attribute(
                f"{self.path}.inputs:{input_}"
            ).set(
                value
            )
        )

    def connect(self, output: str, node: "_Node", input_: str, *, outputs_prefix: str = 'outputs:'):
        self._master.add_action(
            lambda: og.Controller.connect(
                f"{self.path}.{outputs_prefix}{output}",
                f"{node.path}.inputs:{input_}"
            )
        )


class Graph:
    def __init__(self, path: str) -> None:
        self._path: str = path
        self._nodes: list[_Node] = []
        self._actions: list[typing.Callable[[], typing.Any]] = []

    @property
    def path(self) -> str:
        return self._path

    def node(self, name: str, type_: str, **kwargs) -> _Node:
        self._nodes.append(node := _Node(self, name, type_, **kwargs))

        # Keep simulation time in sync across repeated stop/play cycles.
        # Attribute names can vary across Isaac Sim versions, so this is best-effort.
        if type_ == "omni.isaac.core_nodes.IsaacReadSimulationTime":
            def _best_effort_enable_reset_on_stop(node_path: str = node.path) -> None:
                for attr_name in ("resetOnStop", "reset_on_stop", "resetOnStopEnabled"):
                    try:
                        og.Controller.attribute(f"{node_path}.inputs:{attr_name}").set(True)
                        return
                    except Exception:
                        continue

            self.add_action(_best_effort_enable_reset_on_stop)

        return node

    def add_action(self, action: typing.Callable[[], typing.Any]):
        self._actions.append(action)

    def execute_unsafe(self, controller: og.Controller) -> bool:
        controller.edit(
            {"graph_path": self.path, "evaluator_name": "execution"},
            {
                og.Controller.Keys.CREATE_NODES: [(node.name, node.type) for node in self._nodes],
            }
        )

        for action in self._actions:
            action()

        return True

    def execute(self, controller: og.Controller) -> bool:
        try:
            self.execute_unsafe(controller)
        except Exception as e:
            raise e
            return False
        return True
