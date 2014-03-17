within Multirotor.Basics;
model ChassisNRotor
  import Modelica.SIunits.Mass;
  import Modelica.Mechanics.MultiBody.Parts.FixedRotation;
  import Modelica.Mechanics.MultiBody.Parts.Body;
  import Modelica.Mechanics.MultiBody.Interfaces.Frame_a;
  parameter Mass mass = 0.78 "Mass of the chassis (central body)";
  parameter Integer N = 3 "Number of Arms";
  Frame_a frame[N] annotation(Placement(visible = true, transformation(origin = {100,0}, extent = {{-10,-10},{10,10}}, rotation = 0), iconTransformation(origin = {100,0}, extent = {{-10,-10},{10,10}}, rotation = 0)));
  FixedRotation fixedrotation[N - 1](n = array({0,1,0} for i in 1:N - 1), angle = array(360 * i / N for i in 1:N - 1));
  Body body annotation(Placement(visible = true, transformation(origin = {0,0}, extent = {{10,-10},{-10,10}}, rotation = 0)));
equation
  connect(body.frame_a,frame[1]) annotation(Line(points = {{10,0},{99.6951,0},{99.6951,0},{99.6951,0}}));
  for i in 1:N - 1 loop
  connect(body.frame_a,fixedrotation[i].frame_a);
  connect(fixedrotation[i].frame_b,frame[i + 1]);

  end for;
  annotation(Diagram(coordinateSystem(extent = {{-100,-100},{100,100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {0,0})), Icon(coordinateSystem(extent = {{-100,-100},{100,100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {0,0}), graphics = {Ellipse(fillColor = {3,104,255}, fillPattern = FillPattern.CrossDiag, extent = {{100,100},{-100,-100}}, endAngle = 360)}));
end ChassisNRotor;