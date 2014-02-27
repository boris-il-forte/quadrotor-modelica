model Rotor
  import Modelica.Mechanics.Rotational.Interfaces.*;
  import Modelica.Mechanics.MultiBody.Interfaces.*;
  import Modelica.Mechanics.Rotational.Components.Inertia;
  import Modelica.Mechanics.MultiBody.Parts.Body;
  Flange_a flange_a annotation(Placement(visible = true, transformation(origin = {-100,20}, extent = {{-10,-10},{10,10}}, rotation = 0), iconTransformation(origin = {-100,0}, extent = {{-10,-10},{10,10}}, rotation = 0)));
  Inertia inertia1 annotation(Placement(visible = true, transformation(origin = {-40,20}, extent = {{-10,-10},{10,10}}, rotation = 0)));
  Body body1 annotation(Placement(visible = true, transformation(origin = {40,-40}, extent = {{-10,-10},{10,10}}, rotation = 0)));
  Frame_b frame_b annotation(Placement(visible = true, transformation(origin = {0,-100}, extent = {{-17.5,-17.5},{17.5,17.5}}, rotation = 90), iconTransformation(origin = {0,-100}, extent = {{-10,-10},{10,10}}, rotation = -90)));
equation
  connect(frame_b,body1.frame_a) annotation(Line(points = {{0,-100},{0,-39.3548},{29.3548,-39.3548},{29.3548,-39.3548}}));
  connect(inertia1.flange_a,flange_a) annotation(Line(points = {{-50,20},{-100.645,20},{-100.645,20.3226},{-100.645,20.3226}}));
  annotation(Diagram(coordinateSystem(extent = {{-100,-100},{100,100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {2,2})), Icon(coordinateSystem(extent = {{-100,-100},{100,100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {2,2}), graphics = {Rectangle(origin = {-40.26,0.79}, fillColor = {203,203,203}, fillPattern = FillPattern.HorizontalCylinder, extent = {{-21.32,6.05},{21.32,-6.05}}),Rectangle(origin = {44.5982,1.55449}, fillColor = {203,203,203}, fillPattern = FillPattern.HorizontalCylinder, extent = {{-21.32,6.05},{21.32,-6.05}}),Ellipse(origin = {-9.0989,-11.5018}, fillColor = {184,184,184}, fillPattern = FillPattern.Sphere, extent = {{-12.11,-12.63},{34.79,37.79}}, endAngle = 360)}));
end Rotor;