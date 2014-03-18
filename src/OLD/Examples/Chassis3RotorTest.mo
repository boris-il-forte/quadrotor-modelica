within Multirotor.Examples;
model Chassis3RotorTest
  extends Modelica.Icons.Example;
  import Multirotor.Basics.ChassisNRotor;
  import Modelica.Mechanics.MultiBody.Parts.BodyCylinder;
  inner Modelica.Mechanics.MultiBody.World world annotation(Placement(visible = true, transformation(origin = {-40,0}, extent = {{-10,-10},{10,10}}, rotation = 0)));
  ChassisNRotor chassisnrotor1 annotation(Placement(visible = true, transformation(origin = {40,-40}, extent = {{-10,-10},{10,10}}, rotation = 90)));
  BodyCylinder bodycylinder1(r = {10,0,0}) annotation(Placement(visible = true, transformation(origin = {0,0}, extent = {{-10,-10},{10,10}}, rotation = 0)));
  BodyCylinder bodycylinder2(r = {10,0,0}) annotation(Placement(visible = true, transformation(origin = {0,40}, extent = {{-10,-10},{10,10}}, rotation = 0)));
  BodyCylinder bodycylinder3(r = {10,0,0}) annotation(Placement(visible = true, transformation(origin = {0,80}, extent = {{-10,-10},{10,10}}, rotation = 0)));
equation
  connect(bodycylinder3.frame_b,chassisnrotor1.frame[3]) annotation(Line(points = {{10,80},{39.939,80},{39.939,-29.878},{39.939,-29.878}}));
  connect(bodycylinder2.frame_b,chassisnrotor1.frame[2]) annotation(Line(points = {{10,40},{40.5488,40},{40.5488,-29.878},{40.5488,-29.878}}));
  connect(bodycylinder1.frame_b,chassisnrotor1.frame[1]) annotation(Line(points = {{10,0},{40.2439,0},{40.2439,-29.878},{40.2439,-29.878}}));
  connect(bodycylinder1.frame_a,world.frame_b) annotation(Line(points = {{-10,0},{-29.5732,0},{-29.5732,0.304878},{-29.5732,0.304878}}));
  annotation(Icon(coordinateSystem(extent = {{-100,-100},{100,100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {4,4})), Diagram(coordinateSystem(extent = {{-100,-100},{100,100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {4,4})));
end Chassis3RotorTest;