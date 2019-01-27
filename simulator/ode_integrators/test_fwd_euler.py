import unittest
from .forward_euler import ODEScalarFWDEuler

class TestODEFWDEuler(unittest.TestCase):

    def test_constructor_1(self):
        fwd_euler = ODEScalarFWDEuler()
        self.assertEqual(fwd_euler.history_size, 1, "History size should be 1 but is %s"%fwd_euler.history_size)

    def test_constructor_2(self):
        fwd_euler = ODEScalarFWDEuler(init_condition=10)
        self.assertEqual(fwd_euler.get_history(0), 10, "Old value should be 10 but is  %s"%fwd_euler.get_history(0))

    def test_execute(self):
        step_size = 1
        init_condition = 10
        tn = 1
        fwd_euler = ODEScalarFWDEuler(step_size = step_size, init_condition=init_condition)
        yn = fwd_euler.get_history(0)

        def f(**kwargs):
            return tn+yn

        new = fwd_euler.execute(f=f, tn=tn)
        self.assertEqual(new, init_condition + step_size*f())

if __name__ == '__main__':
    unittest.main()
