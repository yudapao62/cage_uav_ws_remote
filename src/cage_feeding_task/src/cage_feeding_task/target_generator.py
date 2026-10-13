from cage_grid.model import CageGridModel


def generate_targets(model, request, start_xy=None):
    return model.targets(request.mode, request.rows, request.cols,
                         request.selected_cell_ids, start_xy,
                         request.feed_relative_alt or 15.0)
