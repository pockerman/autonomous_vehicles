import unittest
from .runge_kutta4 import ODERungeKutta4

class TestODERungeKutta4(unittest.TestCase):

    def test_constructor_1(self):
        rk4 = ODERungeKutta4()
        self.assertEqual(rk4.history_size, 1, "History size should be 1 but is %s"%rk4.history_size)

    def test_constructor_2(self):
        rk4 = ODERungeKutta4(init_condition=10)
        self.assertEqual(rk4.get_history(0), 10, "Old value should be 10 but is  %s"%rk4.get_history(0))

    def test_execute(self):
        step_size = 1
        init_condition = 10
        tn = 1
        rk4 = ODERungeKutta4(step_size = step_size, init_condition=init_condition)

        def f(tn, yn):
            return tn+yn

        kn1 = f(tn, init_condition)
        kn2 = f(tn + 0.5 * step_size, init_condition + 0.5 * step_size * kn1)
        kn3 = f(tn + 0.5 * step_size, init_condition + 0.5 * step_size * kn2)
        kn4 = f(tn + step_size, init_condition + step_size * kn3)

        new = rk4.execute(f=f, tn=tn)
        self.assertEqual(new, init_condition + step_size*((kn1 + 2*kn2 +2.0*kn3 + kn4)/6.0))


if __name__ == '__main__':
    unittest.main()