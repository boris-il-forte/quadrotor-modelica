model brushlessTest
  import Modelica.Electrical.Analog.Sources.ConstantVoltage;
  import Modelica.Electrical.Analog.Basic.Ground;
  ConstantVoltage constantvoltage1 annotation(Placement(visible = true, transformation(origin = {-40,-20}, extent = {{-10,-10},{10,10}}, rotation = -90)));
  Ground ground1 annotation(Placement(visible = true, transformation(origin = {-40,-60}, extent = {{-10,-10},{10,10}}, rotation = 0)));
  BrushlessMotor brushlessmotor1 annotation(Placement(visible = true, transformation(origin = {20,-20}, extent = {{-15,-15},{15,15}}, rotation = 0)));
equation
  connect(constantvoltage1.n,brushlessmotor1.pin2) annotation(Line(points = {{-40,-30},{-40,-28.479},{4.85437,-28.479},{4.85437,-28.479}}));
  connect(brushlessmotor1.pin,constantvoltage1.p) annotation(Line(points = {{5,-11},{-40.4531,-11},{-40.4531,-10.0324},{-40.4531,-10.0324}}));
  connect(ground1.p,constantvoltage1.n) annotation(Line(points = {{-40,-50},{-39.9507,-50},{-39.9507,-29.5931},{-39.9507,-29.5931}}));
  annotation(Icon(coordinateSystem(extent = {{-100,-100},{100,100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {2,2})), Diagram(coordinateSystem(extent = {{-100,-100},{100,100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {2,2})));
end brushlessTest;

