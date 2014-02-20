model Rotor
  Modelica.Mechanics.MultiBody.Interfaces.Frame_a frame_a annotation(Placement(visible = true, transformation(origin = {0,-100}, extent = {{-25,-25},{25,25}}, rotation = -90), iconTransformation(origin = {0,-100}, extent = {{-10,-10},{10,10}}, rotation = -90)));
  Modelica.Mechanics.Rotational.Interfaces.Flange_a flange_a annotation(Placement(visible = true, transformation(origin = {-100,0}, extent = {{-25,-25},{25,25}}, rotation = 0), iconTransformation(origin = {-100,0}, extent = {{-10,-10},{10,10}}, rotation = 0)));
equation
  frame_a.f[3] = flange_a.tau * 1;
  annotation(Diagram(coordinateSystem(extent = {{-100,-100},{100,100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {2,2})), Icon(coordinateSystem(extent = {{-100,-100},{100,100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {2,2}), graphics = {Ellipse(origin = {0,1.05263}, extent = {{-12.1053,-12.6316},{12.1053,12.6316}}, endAngle = 360),Rectangle(origin = {-40.2632,0.789474}, extent = {{-21.3158,6.05263},{21.3158,-6.05263}}),Rectangle(origin = {41.5789,1.05263}, extent = {{-20.5263,6.31579},{20.5263,-6.31579}})}));
end Rotor;

