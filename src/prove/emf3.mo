model emf3
  parameter Boolean useSupport = false "= true, if support flange enabled, otherwise implicitly grounded" annotation(Evaluate = true, HideResult = true, choices(checkBox = true));
  parameter SI.ElectricalTorqueConstant k(start = 1);
  Modelica.Mechanics.Rotational.Interfaces.Flange_b flange_b annotation(Placement(visible = true, transformation(origin = {100,0}, extent = {{-10,-10},{10,10}}, rotation = 0), iconTransformation(origin = {100,0}, extent = {{-10,-10},{10,10}}, rotation = 0)));
  Modelica.Mechanics.Rotational.Interfaces.Support support1 if useSupport annotation(Placement(visible = true, transformation(origin = {0,-100}, extent = {{-10,-10},{10,10}}, rotation = 0), iconTransformation(origin = {-100,0}, extent = {{-10,-10},{10,10}}, rotation = 0)));
  Modelica.Electrical.Analog.Interfaces.Pin P0 annotation(Placement(visible = true, transformation(origin = {-100,60}, extent = {{-10,-10},{10,10}}, rotation = 0), iconTransformation(origin = {-80,100}, extent = {{-10,-10},{10,10}}, rotation = 0)));
  Modelica.Electrical.Analog.Interfaces.Pin P1 annotation(Placement(visible = true, transformation(origin = {-100,0}, extent = {{-10,-10},{10,10}}, rotation = 0), iconTransformation(origin = {0,100}, extent = {{-10,-10},{10,10}}, rotation = 0)));
  Modelica.Electrical.Analog.Interfaces.Pin P2 annotation(Placement(visible = true, transformation(origin = {-100,-60}, extent = {{-10,-10},{10,10}}, rotation = 0), iconTransformation(origin = {80,100}, extent = {{-10,-10},{10,10}}, rotation = 0)));
protected
  Modelica.Mechanics.Rotational.Interfaces.InternalSupport internalsupport1 annotation(Placement(visible = true, transformation(origin = {0,-60}, extent = {{-10,-10},{10,10}}, rotation = 0)));
  Modelica.Mechanics.Rotational.Components.Fixed fixed1 if not useSupport annotation(Placement(visible = true, transformation(origin = {40,-80}, extent = {{-10,-10},{10,10}}, rotation = 0)));
equation
  connect(internalsupport1.flange,support1) annotation(Line(points = {{0,-60},{-0.9146339999999999,-60},{-0.9146339999999999,-96.9512},{-0.9146339999999999,-96.9512}}));
  connect(fixed1.flange,internalsupport1.flange) annotation(Line(points = {{40,-80},{39.3293,-80},{39.3293,-60.061},{0,-60.061},{0,-60.061}}));
  0 = P0.i + P1.i + P2.i;
  0 = vn + e0 - P0.v;
  0 = vn + e1 - P1.v;
  0 = vn + e2 - P2.v;
  k = 2 * kt / 3;
  k0 = k * sin(p * flange_b.phi);
  k1 = k * sin(p * flange_b.phi - 2 / 3 * PI);
  k2 = k * sin(p * flange_b.phi - 4 / 3 * PI);
  e0 = k0 * w;
  e1 = k1 * w;
  e2 = k2 * w;
  w = der(flange_b.phi);
  flange_b.tau = -(k0 * P0.i + k1 * P1.i + k2 * P2.i);
  annotation(Diagram(coordinateSystem(extent = {{-100,-100},{100,100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {2,2})), Icon(coordinateSystem(extent = {{-100,-100},{100,100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {2,2}), graphics = {Rectangle(origin = {-3.96,-0.15}, fillColor = {205,205,205}, fillPattern = FillPattern.HorizontalCylinder, extent = {{-85,10},{-35,-10}}),Rectangle(fillColor = {205,205,205}, fillPattern = FillPattern.HorizontalCylinder, extent = {{35,10},{100,-10}}),Line(origin = {-51.41,54.98}, points = {{-28.4929,39.7136},{27.9806,-39.2013},{28.4738,-39.6946}}, color = {0,0,255}),Line(origin = {-1.6,55.98}, points = {{-0.123305,38.2244},{0.123305,-38.2244},{0.123305,-38.2244}}, color = {0,0,255}),Line(origin = {49.32,54.5}, points = {{31.0727,40.1973},{-31.0727,-40.1973}}, color = {0,0,255}),Ellipse(fillColor = {245,245,245}, fillPattern = FillPattern.Solid, extent = {{-40,40},{40,-40}}, endAngle = 360),Text(origin = {2.34185,-62.2664}, extent = {{-85.94,-22.81},{85.94,22.81}}, textString = "%name")}));
end emf3;