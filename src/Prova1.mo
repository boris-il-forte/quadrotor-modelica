model Prova1
  import Modelica.Mechanics.MultiBody.World;
  import Modelica.Electrical.Analog.Sources.ConstantVoltage;
  import Modelica.Electrical.Analog.Basic.Ground;
  Ground ground1 annotation(Placement(visible = true, transformation(origin = {-80,-80}, extent = {{-10,-10},{10,10}}, rotation = 0)));
  Modelica.Electrical.Analog.Sources.ConstantVoltage constantvoltage1(V = 12) annotation(Placement(visible = true, transformation(origin = {-80,-40}, extent = {{-10,-10},{10,10}}, rotation = -90)));
  inner Modelica.Mechanics.MultiBody.World world(g = 9.81) annotation(Placement(visible = true, transformation(origin = {-80,20}, extent = {{-10,-10},{10,10}}, rotation = 0)));
  Arm arm1 annotation(Placement(visible = true, transformation(origin = {20,20}, extent = {{-25,-25},{25,25}}, rotation = 0)));
equation
  connect(constantvoltage1.n,ground1.p) annotation(Line(points = {{-80,-50},{-80,-68.9474},{-79.47369999999999,-68.9474},{-79.47369999999999,-68.9474}}));
  connect(arm1.pin_n,constantvoltage1.n) annotation(Line(points = {{-5,0},{-3.68421,0},{-3.68421,-50},{-80,-50},{-80,-50}}));
  connect(constantvoltage1.p,arm1.pin_p) annotation(Line(points = {{-80,-30},{-80,11.0526},{-6.31579,11.0526},{-6.31579,11.0526}}));
  connect(arm1.frame_a,world.frame_b) annotation(Line(points = {{-5,20},{-70,20},{-70,20.5263},{-70,20.5263}}));
  annotation(Icon(coordinateSystem(extent = {{-100,-100},{100,100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {2,2})), Diagram(coordinateSystem(extent = {{-100,-100},{100,100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {2,2})));
end Prova1;

