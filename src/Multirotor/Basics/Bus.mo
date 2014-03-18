within Multirotor.Basics;
expandable connector Bus "Bus to connect the controller to the multirotor arm"
  extends Modelica.Icons.SignalSubBus;
  Real sensor "measured angular speed";
  Real control "control signal (voltage)";
end Bus;

