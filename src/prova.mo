model prova
  import Modelica.SIunits.*;
  parameter Angle alpha = 10 "The angle of the rotor blades";
  parameter Real bd[3] = {9.96e-09,2.46e-10,4.33e-07};
  parameter Real bl = 3.88e-07;
  Modelica.Mechanics.Rotational.Interfaces.Flange_a flange_a annotation(Placement(visible = true, transformation(origin = {0,100}, extent = {{-10,-10},{10,10}}, rotation = 0), iconTransformation(origin = {0,100}, extent = {{-10,-10},{10,10}}, rotation = 0)));
  Modelica.Mechanics.Rotational.Components.Inertia inertia annotation(Placement(visible = true, transformation(origin = {-20,40}, extent = {{-10,-10},{10,10}}, rotation = 0)));
  Force L "The lift produced by the rotor";
  Torque Tl "The Drag produced by the rotor";
  AngularVelocity w "The angular velocity of the rotor";
equation
  w = der(inertia.flange_b.phi);
  L = alpha * bl * w ^ 2;
  Tl = bd[1] * w ^ 2 + bd[2] * w ^ 2 * alpha ^ 2 + bd[3] * w * alpha;
  0 = Tl + inertia.flange_b.tau;
  connect(flange_a,inertia.flange_b) annotation(Line(points = {{0,100},{9.451219999999999,100},{9.5122,40.061},{-10,40}}));
  annotation(Icon(coordinateSystem(extent = {{-100,-100},{100,100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {2,2})), Diagram(coordinateSystem(extent = {{-100,-100},{100,100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {2,2})));
end prova;