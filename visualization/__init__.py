"""Optional visualization frontends, loaded only when requested."""
from importlib import import_module

_EXPORTS = {
    'GUIVisualizer': ('visualization.gui_visualizer', 'GUIVisualizer'),
    'UAVXVisualizer': ('visualization.uavx_gui', 'UAVXVisualizer'),
    'start_dashboard': ('visualization.dashboard_server', 'start_dashboard'),
    'update_dashboard_data': ('visualization.dashboard_server', 'update_dashboard_data'),
    'VideoRecorder': ('visualization.video_recorder', 'VideoRecorder'),
    'ChartPlotter': ('visualization.plotter', 'ChartPlotter'),
}
__all__ = list(_EXPORTS)


def __getattr__(name):
    if name not in _EXPORTS:
        raise AttributeError(name)
    module, attribute = _EXPORTS[name]
    value = getattr(import_module(module), attribute)
    globals()[name] = value
    return value
