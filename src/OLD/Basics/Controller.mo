within Multirotor.Basics;
model Controller
  import Modelica.Blocks.Continuous.LimPID;
  import Modelica.Blocks.Interfaces.RealInput;
  import Modelica.Blocks.Math.Gain;
  import Modelica.Blocks.Math.Abs;
  parameter Integer N = 4 "Number of controllers";
  parameter Real K "Gain of controllers";
  parameter Real Ti "Time constat of integrator blocks";
  parameter Real Vmax "Maximum voltage";
  Gain gain[N](k = array((-1) ^ i for i in 1:N)) annotation(Placement(visible = true, transformation(origin = {40,20}, extent = {{-10,-10},{10,10}}, rotation = 0)));
  LimPID PID[N](each controllerType = Modelica.Blocks.Types.SimpleController.PI, each yMin = 0, each k = K, each Ti = Ti, each yMax = Vmax) annotation(Placement(visible = true, transformation(origin = {-20,20}, extent = {{-10,-10},{10,10}}, rotation = 0)));
  Abs abs[N] annotation(Placement(visible = true, transformation(origin = {40,-20}, extent = {{10,-10},{-10,10}}, rotation = 0)));
  Bus bus[N] annotation(Placement(visible = true, transformation(origin = {100,20}, extent = {{-15,-15},{15,15}}, rotation = 90), iconTransformation(origin = {100,20}, extent = {{-10,-10},{10,10}}, rotation = 90)));
  RealInput setPoint[N] annotation(Placement(visible = true, transformation(origin = {-115,25}, extent = {{-15,-15},{15,15}}, rotation = 0), iconTransformation(origin = {-115,25}, extent = {{-15,-15},{15,15}}, rotation = 0)));
equation
  for i in 1:N loop
  connect(setPoint[i],PID[i].u_s);
  connect(bus[N].sensor,abs[N].u);
  connect(abs[N].u,PID[i].u_m);
  connect(PID[i].y,gain[N].u);
  connect(gain[N].y,bus[N].control);

  end for;
  annotation(Diagram(coordinateSystem(extent = {{-100,-100},{100,100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {2,2})), Icon(coordinateSystem(extent = {{-100,-100},{100,100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {2,2}), graphics = {Rectangle(origin = {-0.26,-0.26}, fillColor = {255,255,255}, fillPattern = FillPattern.Backward, extent = {{-99.20999999999999,99.73999999999999},{99.73999999999999,-99.73999999999999}}),Text(origin = {-2.63,6.32}, extent = {{-33.16,15.26},{33.16,-15.26}}, textString = "%name")}));
end Controller;