model Prova3
  inner Modelica.Mechanics.MultiBody.World world annotation(Placement(visible = true, transformation(origin = {-80,20}, extent = {{-10,-10},{10,10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Parts.BodyCylinder bodycylinder1 annotation(Placement(visible = true, transformation(origin = {-40,20}, extent = {{-10,-10},{10,10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Parts.FixedRotation fixedrotation1(angle = 90) annotation(Placement(visible = true, transformation(origin = {0,20}, extent = {{-10,-10},{10,10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Forces.WorldForce force(resolveInFrame = Modelica.Mechanics.MultiBody.Types.ResolveInFrameB.frame_b) annotation(Placement(visible = true, transformation(origin = {60,20}, extent = {{10,-10},{-10,10}}, rotation = 360)));
  Modelica.Blocks.Sources.Constant const(k = 0) annotation(Placement(visible = true, transformation(origin = {0,-20}, extent = {{-10,-10},{10,10}}, rotation = 0)));
  Modelica.Blocks.Sources.Constant constant1(k = 10) annotation(Placement(visible = true, transformation(origin = {60,-60}, extent = {{-10,-10},{10,10}}, rotation = 0)));
equation
  connect(const.y,force.force[3]) annotation(Line(points = {{11,-20},{72.8659,-20},{72.8659,21.0366},{72.8659,21.0366}}));
  connect(const.y,force.force[1]) annotation(Line(points = {{11,-20},{72.8659,-20},{72.8659,20.122},{72.8659,20.122}}));
  connect(constant1.y,force.force[2]) annotation(Line(points = {{71,-60},{88.10980000000001,-60},{88.10980000000001,19.8171},{72.8659,19.8171},{72.8659,19.8171}}));
  connect(force.frame_b,fixedrotation1.frame_b) annotation(Line(points = {{50,20},{9.86717,20},{9.86717,21.2524},{9.86717,21.2524}}));
  connect(fixedrotation1.frame_a,bodycylinder1.frame_b) annotation(Line(points = {{-10,20},{10.111,20},{-30,20},{-30,20}}));
  connect(bodycylinder1.frame_a,world.frame_b) annotation(Line(points = {{-50,20},{-30.0863,20},{-70,20},{-70,20}}));
  annotation(Icon(coordinateSystem(extent = {{-100,-100},{100,100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {2,2})), Diagram(coordinateSystem(extent = {{-100,-100},{100,100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {2,2})));
end Prova3;