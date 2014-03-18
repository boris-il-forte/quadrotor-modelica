within Multirotor.Basics;
model Quadrotor
  inner Modelica.Mechanics.MultiBody.World world annotation(Placement(visible = true, transformation(origin = {-60,80}, extent = {{-10,-10},{10,10}}, rotation = 0)));
  Chassis chassis1 annotation(Placement(visible = true, transformation(origin = {0,0}, extent = {{-26.25,-26.25},{26.25,26.25}}, rotation = 0)));
  Arm arm1 annotation(Placement(visible = true, transformation(origin = {60,0}, extent = {{-22.5,-22.5},{22.5,22.5}}, rotation = 0)));
  Arm arm4 annotation(Placement(visible = true, transformation(origin = {0,60}, extent = {{22.5,-22.5},{-22.5,22.5}}, rotation = -90)));
  Arm arm3 annotation(Placement(visible = true, transformation(origin = {0,-60}, extent = {{-22.5,-22.5},{22.5,22.5}}, rotation = -90)));
  Arm arm2 annotation(Placement(visible = true, transformation(origin = {-60,0}, extent = {{22.5,-22.5},{-22.5,22.5}}, rotation = 0)));
equation
  connect(arm2.frame_a,chassis1.frame_E) annotation(Line(points = {{-37.5,0},{-26.3158,0},{-26.3158,0.526316},{-26.3158,0.526316}}));
  connect(arm3.frame_a,chassis1.frame_S) annotation(Line(points = {{-1.37773e-15,-37.5},{-1.37773e-15,-25.7895},{-1.05263,-25.7895},{-1.05263,-25.7895}}));
  connect(chassis1.frame_N,arm4.frame_a) annotation(Line(points = {{0,26.25},{0,26.25},{1.37773e-15,37.8947},{1.37773e-15,37.5}}));
  connect(chassis1.frame_W,arm1.frame_a) annotation(Line(points = {{26.25,0},{37.3684,0},{37.5,1.05263},{37.5,0}}));
  annotation(Icon(coordinateSystem(extent = {{-100,-100},{100,100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {2,2})), Diagram(coordinateSystem(extent = {{-100,-100},{100,100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {2,2})));
end Quadrotor;