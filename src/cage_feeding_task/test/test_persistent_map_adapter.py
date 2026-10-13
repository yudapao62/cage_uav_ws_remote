import unittest

from cage_feeding_task.persistent_map_adapter import PersistentMapTargetModel


class PersistentMapTargetModelTest(unittest.TestCase):
    def setUp(self):
        self.cells = []
        for row in range(1, 3):
            for col in range(1, 4):
                self.cells.append({
                    'row': row, 'col': col,
                    'cell_id': 'R{:02d}C{:02d}'.format(row, col),
                    'x': col * 10.0 + row, 'y': row * 8.0 + col,
                    'z': 0.0, 'confirmed': True})
        self.model = PersistentMapTargetModel(
            0, 7, self.cells, (-35.363261, 149.165230, 584.0))

    def test_full_grid_uses_estimated_coordinates_and_snake_identity_order(self):
        targets = self.model.targets('FULL_GRID')
        self.assertEqual([value.cell_id for value in targets], [
            'R01C01', 'R01C02', 'R01C03',
            'R02C03', 'R02C02', 'R02C01'])
        self.assertEqual((targets[0].x, targets[0].y), (11.0, 9.0))

    def test_row_order_does_not_change_with_start_position(self):
        targets = self.model.targets('ROW_OR_COLUMN', rows=[1], start_xy=(999, 999))
        self.assertEqual([value.cell_id for value in targets], [
            'R01C01', 'R01C02', 'R01C03'])

    def test_missing_confirmed_cell_fails_closed(self):
        model = PersistentMapTargetModel(
            0, 8, self.cells[:-1], (-35.363261, 149.165230, 584.0))
        with self.assertRaises(ValueError):
            model.targets('FULL_GRID')


if __name__ == '__main__':
    unittest.main()
