# SPDX-FileCopyrightText: Copyright (c) 2024 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: MIT

import time

from isaacsim.core.simulation_manager import SimulationManager


# Concept mechanisim for controling stream rate, only used when not using c++ nodes
class RateLimitedCallback:
    """
    Controls the execution rate of callbacks in the physics simulation.

    This class ensures that a callback function is executed at a specified rate,
    regardless of the physics simulation step frequency. It also provides an
    adaptive rate mechanism that adjusts the execution rate based on the actual
    execution time of the callback.

    Note: This class is only used when not using C++ OGN nodes for streaming.
    """

    def __init__(
        self,
        name: str,
        rate: float,
        fn: callable,
        start_time: float = 0.0,
        adeptive_rate: bool = True,
    ) -> None:
        """
        Initialize a rate-limited callback.

        Args:
            name (str): Name of the callback for identification
            rate (float): Target execution rate in seconds (1/Hz)
            fn (callable): Function to call at the specified rate
            start_time (float): Real world time at which the simulation started
            adeptive_rate (bool): Whether to adaptively adjust the rate based on execution time
        """
        self.name = name
        self.fn = fn  # function to call at rate
        self.rate = rate  # 1/Hz
        self.previous_step_time = 0
        self.accumulated_time = 0
        self.last_exec_time = 0

        self.adeptive_rate = adeptive_rate
        self.start_time = start_time  # real world time at which the simulation started
        self.interval = 3  # seconds between rate adjustments
        self.accumulated_interval_time = 0
        self.exec_count = 0
        self.actual_rate = rate
        self.adj_rate = rate
        self.rates_diff = 0

    def rate_limit(self, dt: float) -> None:
        """
        Execute the callback function at the specified rate.

        This method is called every simulation step, but only executes the callback
        function when enough time has accumulated to match the specified rate.

        When adaptive rate is enabled, it measures the actual execution rate and
        adjusts the target rate to compensate for execution time of the callback.

        Args:
            dt (float): Time step of the physics simulation
        """
        # -> Times here are real world times
        real_time = time.time() - self.start_time
        interval_time = real_time - self.accumulated_interval_time

        # Sample the actual rate each interval, by counting executions
        # Find the difference and set new adjusted rate
        if interval_time >= self.interval and self.adeptive_rate:
            self.accumulated_interval_time = real_time
            interval_rate = self.exec_count / interval_time
            self.actual_rate = (1 / interval_rate) if interval_rate > 0 else self.rate
            self.exec_count = 0

            # Adjust rate to compensate for execution time
            self.rates_diff = self.rate - self.actual_rate
            if abs(self.rate - self.actual_rate) > 0.001:
                self.adj_rate += self.rates_diff

        if not self.adeptive_rate:
            self.adj_rate = self.rate

        # -> Times here are simulated physical times
        elapsed_time = SimulationManager.get_simulation_time() - self.previous_step_time
        self.previous_step_time = SimulationManager.get_simulation_time()

        # Accumulate time until we reach the adjusted rate
        self.accumulated_time += elapsed_time

        # Execute the callback when enough time has accumulated
        if self.accumulated_time >= self.adj_rate:
            self.last_exec_time = self.fn(self.rate, SimulationManager.get_simulation_time())
            self.accumulated_time -= self.adj_rate
            self.exec_count += 1
