model Prova2
  inner Modelica.Mechanics.MultiBody.World world(g = 10) annotation(Placement(visible = true, transformation(origin = {-60,-20}, extent = {{-10,-10},{10,10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Parts.BodyShape bodyshape1(m = 10) annotation(Placement(visible = true, transformation(origin = {-20,-20}, extent = {{-10,-10},{10,10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Parts.Body body1(m = 10) annotation(Placement(visible = true, transformation(origin = {60,0}, extent = {{-10,-10},{10,10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Parts.Body body2(m = 10) annotation(Placement(visible = true, transformation(origin = {60,-40}, extent = {{-10,-10},{10,10}}, rotation = 0)));
equation
  connect(body2.frame_a,bodyshape1.frame_b) annotation(Line(points = {{50,-40},{4.85981,-40},{4.85981,-19.0654},{-9.71963,-19.0654},{-9.71963,-19.0654}}));
  connect(bodyshape1.frame_b,body1.frame_a) annotation(Line(points = {{-10,-20},{5.23364,-20},{5.23364,0},{50.0935,0},{50.0935,0}}));
  connect(bodyshape1.frame_a,world.frame_b) annotation(Line(points = {{-30,-20},{-49.7196,-20},{-49.7196,-19.8131},{-49.7196,-19.8131}}));
  annotation(Icon(coordinateSystem(extent = {{-100,-100},{100,100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {2,2})), Diagram(coordinateSystem(extent = {{-100,-100},{100,100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {2,2})));
end Prova2;

