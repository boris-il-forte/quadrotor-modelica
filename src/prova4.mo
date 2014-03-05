model prova4
  prova prova1 annotation(Placement(visible = true, transformation(origin = {0,-20}, extent = {{-10,-10},{10,10}}, rotation = 0)));
  Modelica.Mechanics.Rotational.Sources.Torque torque1 annotation(Placement(visible = true, transformation(origin = {-20,20}, extent = {{-10,-10},{10,10}}, rotation = 0)));
  Modelica.Blocks.Sources.Constant const(k = 1) annotation(Placement(visible = true, transformation(origin = {-60,20}, extent = {{-10,-10},{10,10}}, rotation = 0)));
equation
  connect(const.y,torque1.tau) annotation(Line(points = {{-49,20},{-32.9268,20},{-32.9268,20.122},{-32.9268,20.122}}));
  connect(torque1.flange,prova1.flange_a) annotation(Line(points = {{-10,20},{0,20},{0,-8.84146},{0,-8.84146}}));
  annotation(Icon(coordinateSystem(extent = {{-100,-100},{100,100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {2,2})), Diagram(coordinateSystem(extent = {{-100,-100},{100,100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {2,2})));
end prova4;