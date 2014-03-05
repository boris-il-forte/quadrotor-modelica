model Rotor
  import Modelica.Mechanics.Rotational.Interfaces.*;
  import Modelica.Mechanics.MultiBody.Interfaces.*;
  import Modelica.Mechanics.Rotational.Components.Inertia;
  import Modelica.Mechanics.MultiBody.Parts.Body;
  import Modelica.SIunits.*;
  parameter Angle alpha = 10 "The angle of the rotor blades";
  parameter Real bd[3] = {9.96e-09,2.46e-10,4.33e-07};
  parameter Real bl = 3.88e-07;
  Flange_a flange annotation(Placement(visible = true, transformation(origin = {-100,20}, extent = {{-10,-10},{10,10}}, rotation = 0), iconTransformation(origin = {-100,0}, extent = {{-10,-10},{10,10}}, rotation = 0)));
  Inertia inertia annotation(Placement(visible = true, transformation(origin = {-40,20}, extent = {{-10,-10},{10,10}}, rotation = 0)));
  InternalSupport internalsupport annotation(Placement(visible = true, transformation(origin = {0,20}, extent = {{-10,-10},{10,10}}, rotation = 0)));
  Frame_b frame annotation(Placement(visible = true, transformation(origin = {0,-100}, extent = {{-17.5,-17.5},{17.5,17.5}}, rotation = 90), iconTransformation(origin = {0,-100}, extent = {{-10,-10},{10,10}}, rotation = -90)));
  Force L[3] "The lift produced by the rotor";
  Torque Tl[3] "The Drag produced by the rotor";
  AngularVelocity w "The angular velocity of the rotor";
equation
  w = der(inertia.flange_b.phi);
  L[2] = alpha * bl * w ^ 2;
  Tl[2] = bd[1] * w ^ 2 + bd[2] * w ^ 2 * alpha ^ 2 + bd[3] * w * alpha;
  internalsupport.tau = Tl[2];
  zeros(3) = L + frame.f;
  zeros(3) = Tl + frame.t;
  connect(internalsupport.flange,inertia.flange_b) annotation(Line(points = {{0,20},{-29.5931,20},{-29.5931,19.9753},{-29.5931,19.9753}}));
  connect(inertia.flange_a,flange) annotation(Line(points = {{-50,20},{-100.645,20},{-100.645,20.3226},{-100.645,20.3226}}));
  annotation(Diagram(coordinateSystem(extent = {{-100,-100},{100,100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {2,2})), Icon(coordinateSystem(extent = {{-100,-100},{100,100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {2,2}), graphics = {Rectangle(origin = {-40.26,0.79}, fillColor = {203,203,203}, fillPattern = FillPattern.HorizontalCylinder, extent = {{-21.32,6.05},{21.32,-6.05}}),Rectangle(origin = {44.5982,1.55449}, fillColor = {203,203,203}, fillPattern = FillPattern.HorizontalCylinder, extent = {{-21.32,6.05},{21.32,-6.05}}),Ellipse(origin = {-9.0989,-11.5018}, fillColor = {184,184,184}, fillPattern = FillPattern.Sphere, extent = {{-12.11,-12.63},{34.79,37.79}}, endAngle = 360)}));
end Rotor;