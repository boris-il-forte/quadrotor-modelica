within Multirotor.Examples;
model Chassis6RotorTest
  extends Modelica.Icons.Example;
  import import Multirotor.Basics.ChassisNRotor;
  import Modelica.Mechanics.MultiBody.Parts.BodyCylinder;
  inner Modelica.Mechanics.MultiBody.World world annotation(Placement(visible = true, transformation(origin = {-80,-40}, extent = {{-10,-10},{10,10}}, rotation = 0)));
  ChassisNRotor chassisnrotor1(N = 6) annotation(Placement(visible = true, transformation(origin = {40,-80}, extent = {{-10,-10},{10,10}}, rotation = 90)));
  BodyCylinder bodycylinder1(r = {10,0,0}) annotation(Placement(visible = true, transformation(origin = {-40,-40}, extent = {{-10,-10},{10,10}}, rotation = 0)));
  BodyCylinder bodycylinder2(r = {10,0,0}) annotation(Placement(visible = true, transformation(origin = {-40,0}, extent = {{-10,-10},{10,10}}, rotation = 0)));
  BodyCylinder bodycylinder3(r = {10,0,0}) annotation(Placement(visible = true, transformation(origin = {-40,40}, extent = {{-10,-10},{10,10}}, rotation = 0)));
  BodyCylinder bodycylinder4(r = {10,0,0}) annotation(Placement(visible = true, transformation(origin = {-40,80}, extent = {{-10,-10},{10,10}}, rotation = 0)));
  BodyCylinder bodycylinder5(r = {10,0,0}) annotation(Placement(visible = true, transformation(origin = {80,80}, extent = {{-10,-10},{10,10}}, rotation = 0)));
  BodyCylinder bodycylinder6(r = {10,0,0}) annotation(Placement(visible = true, transformation(origin = {80,40}, extent = {{-10,-10},{10,10}}, rotation = 0)));
equation
  connect(chassisnrotor1.frame[4],bodycylinder4.frame_b) annotation(Line(points = {{40,-70},{40,67.6829},{14.6341,67.6829},{14.6341,79.5732},{-30,79.5732},{-30,80}}));
  connect(chassisnrotor1.frame[3],bodycylinder3.frame_b) annotation(Line(points = {{40,-70},{40,24.3902},{16.4634,24.3902},{16.4634,40.2439},{-30,40.2439},{-30,40}}));
  connect(bodycylinder2.frame_b,chassisnrotor1.frame[2]) annotation(Line(points = {{-30,0},{0.304878,0},{0.304878,-70},{40,-70}}));
  connect(world.frame_b,bodycylinder1.frame_a) annotation(Line(points = {{-70,-40},{-10.061,-40},{-50,-40},{-50,-40}}));
  connect(bodycylinder1.frame_b,chassisnrotor1.frame[1]) annotation(Line(points = {{-30,-40},{-0.304878,-40},{-0.304878,-70},{40,-70}}));
  connect(chassisnrotor1.frame[6],bodycylinder6.frame_b) annotation(Line(points = {{40,-70},{40,30.1829},{96.9512,30.1829},{96.9512,39.3293},{90,39.3293},{90,40}}));
  connect(chassisnrotor1.frame[5],bodycylinder5.frame_b) annotation(Line(points = {{40,-70},{40,68.2927},{95.122,68.2927},{95.122,80.1829},{90,80.1829},{90,80}}));
  annotation(Icon(coordinateSystem(extent = {{-100,-100},{100,100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {4,4})), Diagram(coordinateSystem(extent = {{-100,-100},{100,100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {4,4})));
end Chassis6RotorTest;