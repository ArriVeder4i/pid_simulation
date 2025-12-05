import matplotlib

# ВАЖНО: Эта строчка отключает попытку открыть окно и лечит ошибку TclError
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import time


class PIDController:
    """
    Классический PID-регулятор.
    Kp - пропорциональный коэффициент (сила реакции на ошибку)
    Ki - интегральный коэффициент (накопление ошибки, убирает недогрев)
    Kd - дифференциальный коэффициент (предсказание, гасит колебания)
    """

    def __init__(self, Kp, Ki, Kd, output_limits=(0, 100)):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.min_out, self.max_out = output_limits

        self.prev_error = 0
        self.integral = 0

    def update(self, setpoint, measured_value, dt):
        # 1. Вычисляем ошибку (Сколько не хватает до цели?)
        error = setpoint - measured_value

        # 2. Пропорциональная часть (P)
        P = self.Kp * error

        # 3. Интегральная часть (I) - накапливаем ошибку
        self.integral += error * dt
        # Ограничим интеграл (Anti-windup)
        self.integral = max(min(self.integral, 100), -100)
        I = self.Ki * self.integral

        # 4. Дифференциальная часть (D) - скорость изменения ошибки
        derivative = (error - self.prev_error) / dt
        D = self.Kd * derivative

        # 5. Итоговый управляющий сигнал
        output = P + I + D

        # Запоминаем ошибку
        self.prev_error = error

        # Ограничиваем выход (0-100%)
        return max(min(output, self.max_out), self.min_out)


def simulation():
    # --- НАСТРОЙКИ СИМУЛЯЦИИ ---
    target_temp = 80.0  # Целевая температура
    current_temp = 20.0  # Начальная температура
    dt = 0.1  # Шаг времени
    total_time = 100  # Длительность

    # Настройки PID
    pid = PIDController(Kp=2.0, Ki=0.1, Kd=0.05)

    times = []
    temps = []
    powers = []
    targets = []

    print(f"🚀 Запуск симуляции нагрева до {target_temp}°C...")

    # --- ГЛАВНЫЙ ЦИКЛ ---
    for i in range(int(total_time / dt)):
        t = i * dt

        # 1. PID решает, какую мощность дать
        power = pid.update(target_temp, current_temp, dt)

        # 2. Физика нагрева
        heating = (power * 0.1) * dt
        cooling = (current_temp - 20) * 0.02 * dt

        current_temp += heating - cooling

        times.append(t)
        temps.append(current_temp)
        powers.append(power)
        targets.append(target_temp)

    # --- ВИЗУАЛИЗАЦИЯ ---
    print("Генерация графика...")
    plt.figure(figsize=(12, 8))

    # График 1: Температура
    plt.subplot(2, 1, 1)
    plt.plot(times, temps, label='Текущая температура', color='red', linewidth=2)
    plt.plot(times, targets, label='Цель (Setpoint)', color='green', linestyle='--')
    plt.title('Нагрев чайника (PID-регулирование)')
    plt.ylabel('Температура (°C)')
    plt.legend()
    plt.grid(True)

    # График 2: Мощность
    plt.subplot(2, 1, 2)
    plt.plot(times, powers, label='Мощность нагревателя (%)', color='blue')
    plt.xlabel('Время (сек)')
    plt.ylabel('Мощность (%)')
    plt.ylim(0, 105)
    plt.grid(True)

    # Сохраняем файл вместо показа окна
    plt.tight_layout()
    plt.savefig('pid_result.png')
    # Обязательно закрываем, чтобы освободить память
    plt.close()

    print("✅ Симуляция завершена! Открой файл pid_result.png, чтобы увидеть результат.")


if __name__ == "__main__":
    simulation()