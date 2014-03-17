within Multirotor.Examples;
model ChassisTest
  extends Modelica.Icons.Example;
  import Multirotor.Basics.Chassis;
  Chassis chassis1 annotation(Placement(visible = true, transformation(origin = {0,0}, extent = {{-10,-10},{10,10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Parts.BodyCylinder bodycylinder1(r = {10,0,0}) annotation(Placement(visible = true, transformation(origin = {-40,0}, extent = {{-10,-10},{10,10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Parts.BodyCylinder bodycylinder2(r = {10,0,0}) annotation(Placement(visible = true, transformation(origin = {0,40}, extent = {{-10,-10},{10,10}}, rotation = -90)));
  Modelica.Mechanics.MultiBody.Parts.BodyCylinder bodycylinder3(r = {10,0,0}) annotation(Placement(visible = true, transformation(origin = {40,0}, extent = {{10,-10},{-10,10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Parts.BodyCylinder bodycylinder4(r = {10,0,0}) annotation(Placement(visible = true, transformation(origin = {0,-40}, extent = {{-10,-10},{10,10}}, rotation = 90)));
  inner Modelica.Mechanics.MultiBody.World world annotation(Placement(visible = true, transformation(origin = {-80,0}, extent = {{-10,-10},{10,10}}, rotation = 0)));
equation
  connect(world.frame_b,bodycylinder1.frame_a) annotation(Line(points = {{-70,0},{-50.0778,0},{-50.0778,0.311042},{-50.0778,0.311042}}));
  connect(bodycylinder4.frame_b,chassis1.frame_S) annotation(Line(points = {{6.12323e-16,-30},{6.12323e-16,-9.953340000000001},{-0.622084,-9.953340000000001},{-0.622084,-9.953340000000001}}));
  connect(bodycylinder3.frame_b,chassis1.frame_W) annotation(Line(points = {{30,0},{10.2644,0},{10.2644,0},{10,0}}));
  connect(chassis1.frame_N,bodycylinder2.frame_b) annotation(Line(points = {{0,10},{0,10},{0,30.1711},{0,30.1711}}));
  connect(bodycylinder1.frame_b,chassis1.frame_E) annotation(Line(points = {{-30,0},{-10.8865,0},{-10.8865,0},{-10.8865,0}}));
  annotation(Icon(coordinateSystem(extent = {{-100,-100},{100,100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {4,4})), Diagram(coordinateSystem(extent = {{-100,-100},{100,100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {4,4})));
end ChassisTest;

