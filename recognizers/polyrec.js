/**
 * The PolyRec Recognizer
 * 
 * This version was slightly modified for evaluation purposes:
 *    1. All example gestures have been removed;
 *    2. The AddTemplate function was renamed into AddGesture;
 *    3. The PointCloud constructor has been extended with resamplingPoints; TODO ?
 *    4. The PDollarRecognizer constructor has been extended with resamplingPoints; TODO ?
 *    5. All functions and global variables have been pre-fixed with 'PolyRec_'.
 *    6. Performance measurements were added in the AddGesture and Recognize functions.
 *
 * Author of the modifications:
 *    Arthur Sluÿters
 *    Université catholique de Louvain
 *    Louvain Research Institute in Management and Organizations
 *    Louvain-la-Neuve, Belgium
 *    arthur.sluyters@uclouvain.be
 * 
 * BSD 3-Clause License
 *
 * PolyRec Project
 * Copyright (c) 2015-2017, Vittorio Fuccella - CLUE Lab - http://cluelab.di.unisa.it
 * All rights reserved. Includes a reference implementation of the following:
 *
 * * Vittorio Fuccella, Gennaro Costagliola. "Unistroke PolyRec_Gesture Recognition
 * Through PolyRec_Polyline Approximation and Alignment". In Proceedings of the 33rd
 * annual ACM conference on Human factors in computing systems (CHI '15).
 * April 18-23, 2015, Seoul, Republic of Korea.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *     * Neither the name of the PolyRec Project nor the
 *     names of its contributors may be used to endorse or promote products
 *     derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

//classe PolyRec_Point
function PolyRec_Point(x,y)
{
    this.X = x;
    this.Y = y;
}

//classe PolyRec_Rectangle
function PolyRec_Rectangle(x, y, width, height)
{
	this.X = x;
	this.Y = y;
	this.Width = width;
	this.Height = height;
}

//classe PolyRec_Vector
function PolyRec_Vector(intensity, angle)
{
    this.Intensity = intensity;
    this.PolyRec_Angle = angle;
}

//classe PolyRec_Result
function PolyRec_Result(name, score, ms)
{
	this.Name = name;
	this.Score = Math.round(score * 10000) / 100.0;
	this.Time = ms;
}

//classe PolyRec_Polyline
function PolyRec_Polyline(gesture, indexes)
{
    this.PolyRec_Gesture = gesture; 
    this.Indexes = indexes;
    this.Lengths = PolyRec_CalculateLengthsPoly(gesture.Points, indexes);
}

//classe PolyRec_Gesture
function PolyRec_Gesture(name, points)
{
    this.Name = name;
    this.Points = points;
    this.Centroid = PolyRec_CalculateCentroid(points);
    this.Lengths = PolyRec_CalculateLengths(points);
    this.BoundingBox = PolyRec_CalculateBoundingBox(points);
    this.RotInv = false;
    this.PointersNum = 1;
}

//
// PolyRecognizer constants
//
const DPR_PARAMS = {FParam: 26.0, SParam: 22.0};
const ANGLE_ROTATION_INVARIANT = 45;
const ANGLE_ROTATION_SENSITIVE = 25;
const ANGLE_STEP = 2;
const PHI = 0.5 * ( -1.0 + Math.sqrt(5.0) );
const GAP_COST = 0.4;
const BALANCE = 0.6;
const SLOPE_TRESHOLD = DPR_PARAMS.FParam;

//
// PolyRecognizer class
//
function PolyRecognizer() // constructor
{
    this.DataSet = new Array();

    // if(def){
    // var gest0 = new PolyRec_Gesture("arrow", new Array( new PolyRec_Point(69,228),new PolyRec_Point(67,228),new PolyRec_Point(70,225),new PolyRec_Point(73,223),new PolyRec_Point(75,222),new PolyRec_Point(80,218),new PolyRec_Point(82,216),new PolyRec_Point(85,214),new PolyRec_Point(92,209),new PolyRec_Point(99,205),new PolyRec_Point(106,201),new PolyRec_Point(114,196),new PolyRec_Point(122,193),new PolyRec_Point(130,188),new PolyRec_Point(138,185),new PolyRec_Point(145,181),new PolyRec_Point(148,180),new PolyRec_Point(154,177),new PolyRec_Point(157,175),new PolyRec_Point(162,173),new PolyRec_Point(164,172),new PolyRec_Point(167,171),new PolyRec_Point(164,170),new PolyRec_Point(161,169),new PolyRec_Point(155,170),new PolyRec_Point(148,170),new PolyRec_Point(144,170),new PolyRec_Point(142,169),new PolyRec_Point(138,169),new PolyRec_Point(133,170),new PolyRec_Point(131,170),new PolyRec_Point(133,170),new PolyRec_Point(139,168),new PolyRec_Point(148,167),new PolyRec_Point(157,163),new PolyRec_Point(169,159),new PolyRec_Point(177,154),new PolyRec_Point(181,153),new PolyRec_Point(186,152),new PolyRec_Point(186,154),new PolyRec_Point(181,159),new PolyRec_Point(171,170),new PolyRec_Point(162,181),new PolyRec_Point(157,189),new PolyRec_Point(155,192),new PolyRec_Point(149,201),new PolyRec_Point(144,205),new PolyRec_Point(142,205)));
    // var gest1 = new PolyRec_Gesture("caret", new Array( new PolyRec_Point(73,243),new PolyRec_Point(74,241),new PolyRec_Point(76,237),new PolyRec_Point(77,235),new PolyRec_Point(79,231),new PolyRec_Point(82,224),new PolyRec_Point(83,222),new PolyRec_Point(85,218),new PolyRec_Point(88,210),new PolyRec_Point(92,201),new PolyRec_Point(96,192),new PolyRec_Point(100,181),new PolyRec_Point(104,169),new PolyRec_Point(106,163),new PolyRec_Point(110,152),new PolyRec_Point(111,147),new PolyRec_Point(114,138),new PolyRec_Point(119,127),new PolyRec_Point(121,123),new PolyRec_Point(123,120),new PolyRec_Point(125,119),new PolyRec_Point(128,121),new PolyRec_Point(133,127),new PolyRec_Point(136,135),new PolyRec_Point(143,149),new PolyRec_Point(151,169),new PolyRec_Point(160,188),new PolyRec_Point(167,205),new PolyRec_Point(173,220),new PolyRec_Point(176,232),new PolyRec_Point(178,236),new PolyRec_Point(177,243)));
    // var gest2 = new PolyRec_Gesture("check", new Array( new PolyRec_Point(86,195),new PolyRec_Point(88,199),new PolyRec_Point(89,201),new PolyRec_Point(91,205),new PolyRec_Point(95,209),new PolyRec_Point(96,211),new PolyRec_Point(100,217),new PolyRec_Point(102,219),new PolyRec_Point(105,223),new PolyRec_Point(108,227),new PolyRec_Point(110,230),new PolyRec_Point(112,232),new PolyRec_Point(114,234),new PolyRec_Point(114,230),new PolyRec_Point(113,226),new PolyRec_Point(113,222),new PolyRec_Point(115,211),new PolyRec_Point(117,203),new PolyRec_Point(120,190),new PolyRec_Point(124,180),new PolyRec_Point(127,176),new PolyRec_Point(134,165),new PolyRec_Point(142,155),new PolyRec_Point(150,146),new PolyRec_Point(156,141),new PolyRec_Point(159,138),new PolyRec_Point(161,134)));
    // var gest3 = new PolyRec_Gesture("circle", new Array( new PolyRec_Point(119,132),new PolyRec_Point(115,133),new PolyRec_Point(112,134),new PolyRec_Point(108,136),new PolyRec_Point(104,138),new PolyRec_Point(98,143),new PolyRec_Point(92,149),new PolyRec_Point(90,153),new PolyRec_Point(88,157),new PolyRec_Point(85,166),new PolyRec_Point(84,175),new PolyRec_Point(85,184),new PolyRec_Point(86,193),new PolyRec_Point(90,202),new PolyRec_Point(95,209),new PolyRec_Point(98,212),new PolyRec_Point(104,217),new PolyRec_Point(116,221),new PolyRec_Point(123,221),new PolyRec_Point(131,219),new PolyRec_Point(136,217),new PolyRec_Point(147,213),new PolyRec_Point(153,208),new PolyRec_Point(156,205),new PolyRec_Point(162,193),new PolyRec_Point(165,181),new PolyRec_Point(163,168),new PolyRec_Point(160,158),new PolyRec_Point(157,154),new PolyRec_Point(148,141),new PolyRec_Point(145,138),new PolyRec_Point(135,130),new PolyRec_Point(133,128),new PolyRec_Point(125,129),new PolyRec_Point(119,131),new PolyRec_Point(114,135)));
    // var gest4 = new PolyRec_Gesture("delete_mark", new Array( new PolyRec_Point(115,137),new PolyRec_Point(115,139),new PolyRec_Point(116,142),new PolyRec_Point(117,144),new PolyRec_Point(120,150),new PolyRec_Point(126,158),new PolyRec_Point(129,162),new PolyRec_Point(133,166),new PolyRec_Point(141,174),new PolyRec_Point(149,181),new PolyRec_Point(156,188),new PolyRec_Point(161,193),new PolyRec_Point(163,194),new PolyRec_Point(166,197),new PolyRec_Point(167,199),new PolyRec_Point(167,201),new PolyRec_Point(165,202),new PolyRec_Point(162,202),new PolyRec_Point(157,201),new PolyRec_Point(147,196),new PolyRec_Point(135,194),new PolyRec_Point(122,192),new PolyRec_Point(111,191),new PolyRec_Point(102,195),new PolyRec_Point(97,198),new PolyRec_Point(91,200),new PolyRec_Point(89,201),new PolyRec_Point(92,198),new PolyRec_Point(96,192),new PolyRec_Point(104,185),new PolyRec_Point(112,173),new PolyRec_Point(115,170),new PolyRec_Point(122,161),new PolyRec_Point(125,158),new PolyRec_Point(135,147),new PolyRec_Point(141,141),new PolyRec_Point(146,136)));
    // var gest5 = new PolyRec_Gesture("left_curly_brace", new Array( new PolyRec_Point(143,125),new PolyRec_Point(141,125),new PolyRec_Point(139,125),new PolyRec_Point(135,126),new PolyRec_Point(133,126),new PolyRec_Point(131,127),new PolyRec_Point(129,127),new PolyRec_Point(125,129),new PolyRec_Point(123,131),new PolyRec_Point(119,134),new PolyRec_Point(115,137),new PolyRec_Point(112,139),new PolyRec_Point(109,142),new PolyRec_Point(107,145),new PolyRec_Point(106,147),new PolyRec_Point(105,149),new PolyRec_Point(107,150),new PolyRec_Point(110,150),new PolyRec_Point(114,149),new PolyRec_Point(118,150),new PolyRec_Point(122,149),new PolyRec_Point(124,149),new PolyRec_Point(128,151),new PolyRec_Point(132,152),new PolyRec_Point(135,155),new PolyRec_Point(137,158),new PolyRec_Point(138,160),new PolyRec_Point(138,166),new PolyRec_Point(132,175),new PolyRec_Point(127,178),new PolyRec_Point(124,181),new PolyRec_Point(115,187),new PolyRec_Point(111,190),new PolyRec_Point(105,193),new PolyRec_Point(108,191),new PolyRec_Point(111,190),new PolyRec_Point(118,188),new PolyRec_Point(122,187),new PolyRec_Point(129,189),new PolyRec_Point(131,190),new PolyRec_Point(134,196),new PolyRec_Point(133,203),new PolyRec_Point(132,205),new PolyRec_Point(124,218),new PolyRec_Point(120,226),new PolyRec_Point(118,236),new PolyRec_Point(120,245),new PolyRec_Point(125,251),new PolyRec_Point(131,250)));
    // var gest6 = new PolyRec_Gesture("left_sq_bracket", new Array( new PolyRec_Point(144,122),new PolyRec_Point(141,122),new PolyRec_Point(139,122),new PolyRec_Point(137,121),new PolyRec_Point(134,121),new PolyRec_Point(131,121),new PolyRec_Point(129,122),new PolyRec_Point(125,122),new PolyRec_Point(122,123),new PolyRec_Point(119,123),new PolyRec_Point(116,123),new PolyRec_Point(114,123),new PolyRec_Point(112,124),new PolyRec_Point(109,123),new PolyRec_Point(107,124),new PolyRec_Point(105,125),new PolyRec_Point(103,128),new PolyRec_Point(103,131),new PolyRec_Point(103,135),new PolyRec_Point(103,141),new PolyRec_Point(104,143),new PolyRec_Point(105,153),new PolyRec_Point(106,159),new PolyRec_Point(106,163),new PolyRec_Point(107,171),new PolyRec_Point(107,174),new PolyRec_Point(108,183),new PolyRec_Point(108,192),new PolyRec_Point(108,199),new PolyRec_Point(108,207),new PolyRec_Point(108,213),new PolyRec_Point(108,219),new PolyRec_Point(108,222),new PolyRec_Point(108,225),new PolyRec_Point(108,227),new PolyRec_Point(110,229),new PolyRec_Point(112,230),new PolyRec_Point(115,231),new PolyRec_Point(121,232),new PolyRec_Point(128,234),new PolyRec_Point(134,234),new PolyRec_Point(139,235),new PolyRec_Point(145,237),new PolyRec_Point(150,236)));
    // var gest7 = new PolyRec_Gesture("pigtail", new Array( new PolyRec_Point(65,219),new PolyRec_Point(68,220),new PolyRec_Point(70,221),new PolyRec_Point(73,220),new PolyRec_Point(75,220),new PolyRec_Point(80,220),new PolyRec_Point(84,219),new PolyRec_Point(89,217),new PolyRec_Point(91,216),new PolyRec_Point(94,215),new PolyRec_Point(99,213),new PolyRec_Point(105,211),new PolyRec_Point(108,209),new PolyRec_Point(113,207),new PolyRec_Point(116,205),new PolyRec_Point(122,201),new PolyRec_Point(130,193),new PolyRec_Point(135,190),new PolyRec_Point(141,182),new PolyRec_Point(145,177),new PolyRec_Point(147,175),new PolyRec_Point(151,167),new PolyRec_Point(152,161),new PolyRec_Point(153,159),new PolyRec_Point(152,152),new PolyRec_Point(151,148),new PolyRec_Point(146,142),new PolyRec_Point(142,141),new PolyRec_Point(134,143),new PolyRec_Point(132,145),new PolyRec_Point(123,153),new PolyRec_Point(118,163),new PolyRec_Point(115,175),new PolyRec_Point(115,179),new PolyRec_Point(115,193),new PolyRec_Point(116,198),new PolyRec_Point(119,215),new PolyRec_Point(126,226),new PolyRec_Point(135,233),new PolyRec_Point(147,234),new PolyRec_Point(160,232),new PolyRec_Point(169,228)));
    // var gest8 = new PolyRec_Gesture("question_mark", new Array( new PolyRec_Point(99,141),new PolyRec_Point(100,139),new PolyRec_Point(101,137),new PolyRec_Point(103,134),new PolyRec_Point(105,132),new PolyRec_Point(108,129),new PolyRec_Point(111,128),new PolyRec_Point(113,126),new PolyRec_Point(116,124),new PolyRec_Point(119,122),new PolyRec_Point(122,121),new PolyRec_Point(126,120),new PolyRec_Point(130,122),new PolyRec_Point(134,122),new PolyRec_Point(138,124),new PolyRec_Point(141,127),new PolyRec_Point(144,131),new PolyRec_Point(146,133),new PolyRec_Point(149,138),new PolyRec_Point(150,140),new PolyRec_Point(152,144),new PolyRec_Point(152,150),new PolyRec_Point(152,152),new PolyRec_Point(151,156),new PolyRec_Point(149,161),new PolyRec_Point(146,164),new PolyRec_Point(143,166),new PolyRec_Point(136,169),new PolyRec_Point(132,169),new PolyRec_Point(129,169),new PolyRec_Point(126,169),new PolyRec_Point(124,168),new PolyRec_Point(121,167),new PolyRec_Point(119,167),new PolyRec_Point(118,170),new PolyRec_Point(118,173),new PolyRec_Point(118,178),new PolyRec_Point(120,186),new PolyRec_Point(120,188),new PolyRec_Point(121,195),new PolyRec_Point(122,197),new PolyRec_Point(122,201),new PolyRec_Point(122,204),new PolyRec_Point(122,208),new PolyRec_Point(121,210),new PolyRec_Point(121,212)));
    // var gest9 = new PolyRec_Gesture("rectangle", new Array( new PolyRec_Point(66,162),new PolyRec_Point(64,165),new PolyRec_Point(64,167),new PolyRec_Point(64,169),new PolyRec_Point(64,172),new PolyRec_Point(64,174),new PolyRec_Point(64,179),new PolyRec_Point(65,182),new PolyRec_Point(65,185),new PolyRec_Point(65,189),new PolyRec_Point(66,192),new PolyRec_Point(66,198),new PolyRec_Point(67,202),new PolyRec_Point(67,205),new PolyRec_Point(67,211),new PolyRec_Point(68,216),new PolyRec_Point(68,219),new PolyRec_Point(68,223),new PolyRec_Point(68,226),new PolyRec_Point(67,228),new PolyRec_Point(67,231),new PolyRec_Point(67,234),new PolyRec_Point(67,237),new PolyRec_Point(67,239),new PolyRec_Point(69,241),new PolyRec_Point(71,240),new PolyRec_Point(74,238),new PolyRec_Point(77,238),new PolyRec_Point(85,237),new PolyRec_Point(93,238),new PolyRec_Point(102,239),new PolyRec_Point(111,237),new PolyRec_Point(122,237),new PolyRec_Point(132,238),new PolyRec_Point(143,239),new PolyRec_Point(153,240),new PolyRec_Point(156,240),new PolyRec_Point(167,241),new PolyRec_Point(176,243),new PolyRec_Point(188,245),new PolyRec_Point(192,245),new PolyRec_Point(199,247),new PolyRec_Point(201,247),new PolyRec_Point(203,246),new PolyRec_Point(203,243),new PolyRec_Point(200,237),new PolyRec_Point(197,229),new PolyRec_Point(196,225),new PolyRec_Point(191,213),new PolyRec_Point(187,199),new PolyRec_Point(183,183),new PolyRec_Point(181,168),new PolyRec_Point(179,157),new PolyRec_Point(178,153),new PolyRec_Point(177,148),new PolyRec_Point(175,147),new PolyRec_Point(173,148),new PolyRec_Point(170,148),new PolyRec_Point(162,146),new PolyRec_Point(151,142),new PolyRec_Point(148,142),new PolyRec_Point(136,140),new PolyRec_Point(133,141),new PolyRec_Point(120,141),new PolyRec_Point(117,141),new PolyRec_Point(101,144),new PolyRec_Point(90,147)));
    // var gest10 = new PolyRec_Gesture("right_curly_brace", new Array( new PolyRec_Point(102,139),new PolyRec_Point(105,138),new PolyRec_Point(107,138),new PolyRec_Point(111,138),new PolyRec_Point(113,137),new PolyRec_Point(118,137),new PolyRec_Point(122,137),new PolyRec_Point(127,137),new PolyRec_Point(132,138),new PolyRec_Point(136,140),new PolyRec_Point(139,141),new PolyRec_Point(142,143),new PolyRec_Point(144,145),new PolyRec_Point(145,148),new PolyRec_Point(146,151),new PolyRec_Point(145,154),new PolyRec_Point(144,156),new PolyRec_Point(141,159),new PolyRec_Point(140,161),new PolyRec_Point(136,163),new PolyRec_Point(129,168),new PolyRec_Point(125,171),new PolyRec_Point(121,172),new PolyRec_Point(118,174),new PolyRec_Point(115,175),new PolyRec_Point(113,176),new PolyRec_Point(111,178),new PolyRec_Point(112,180),new PolyRec_Point(115,182),new PolyRec_Point(118,185),new PolyRec_Point(123,188),new PolyRec_Point(130,189),new PolyRec_Point(137,190),new PolyRec_Point(141,191),new PolyRec_Point(147,190),new PolyRec_Point(150,188),new PolyRec_Point(152,187),new PolyRec_Point(151,184),new PolyRec_Point(149,183),new PolyRec_Point(142,183),new PolyRec_Point(134,184),new PolyRec_Point(131,185),new PolyRec_Point(122,191),new PolyRec_Point(117,196),new PolyRec_Point(112,200),new PolyRec_Point(111,202),new PolyRec_Point(110,204),new PolyRec_Point(113,208),new PolyRec_Point(119,211),new PolyRec_Point(123,212),new PolyRec_Point(126,214),new PolyRec_Point(132,219),new PolyRec_Point(136,226),new PolyRec_Point(138,234),new PolyRec_Point(135,241),new PolyRec_Point(128,247),new PolyRec_Point(126,249),new PolyRec_Point(112,255),new PolyRec_Point(103,257),new PolyRec_Point(98,258)));
    // var gest11 = new PolyRec_Gesture("right_sq_bracket", new Array( new PolyRec_Point(110,144),new PolyRec_Point(112,146),new PolyRec_Point(115,146),new PolyRec_Point(118,145),new PolyRec_Point(120,145),new PolyRec_Point(124,145),new PolyRec_Point(128,145),new PolyRec_Point(132,145),new PolyRec_Point(136,147),new PolyRec_Point(139,147),new PolyRec_Point(142,150),new PolyRec_Point(145,154),new PolyRec_Point(147,158),new PolyRec_Point(148,164),new PolyRec_Point(148,171),new PolyRec_Point(149,174),new PolyRec_Point(149,181),new PolyRec_Point(149,185),new PolyRec_Point(149,194),new PolyRec_Point(149,206),new PolyRec_Point(149,217),new PolyRec_Point(149,225),new PolyRec_Point(149,227),new PolyRec_Point(151,233),new PolyRec_Point(149,233),new PolyRec_Point(145,231),new PolyRec_Point(137,230),new PolyRec_Point(133,228),new PolyRec_Point(121,226),new PolyRec_Point(108,225),new PolyRec_Point(100,226),new PolyRec_Point(95,227),new PolyRec_Point(92,228)));
    // var gest12 = new PolyRec_Gesture("star", new Array( new PolyRec_Point(77,261),new PolyRec_Point(79,259),new PolyRec_Point(80,257),new PolyRec_Point(82,254),new PolyRec_Point(83,251),new PolyRec_Point(85,248),new PolyRec_Point(87,244),new PolyRec_Point(87,242),new PolyRec_Point(88,239),new PolyRec_Point(90,233),new PolyRec_Point(92,226),new PolyRec_Point(95,220),new PolyRec_Point(97,212),new PolyRec_Point(99,206),new PolyRec_Point(101,199),new PolyRec_Point(102,195),new PolyRec_Point(104,184),new PolyRec_Point(105,177),new PolyRec_Point(108,166),new PolyRec_Point(109,158),new PolyRec_Point(113,147),new PolyRec_Point(115,140),new PolyRec_Point(116,137),new PolyRec_Point(118,130),new PolyRec_Point(120,127),new PolyRec_Point(121,124),new PolyRec_Point(123,127),new PolyRec_Point(124,130),new PolyRec_Point(124,133),new PolyRec_Point(127,141),new PolyRec_Point(130,149),new PolyRec_Point(134,159),new PolyRec_Point(136,164),new PolyRec_Point(141,177),new PolyRec_Point(148,193),new PolyRec_Point(155,208),new PolyRec_Point(159,216),new PolyRec_Point(164,224),new PolyRec_Point(165,226),new PolyRec_Point(166,229),new PolyRec_Point(167,231),new PolyRec_Point(167,235),new PolyRec_Point(166,240),new PolyRec_Point(165,245),new PolyRec_Point(163,245),new PolyRec_Point(160,245),new PolyRec_Point(152,239),new PolyRec_Point(140,233),new PolyRec_Point(126,224),new PolyRec_Point(110,211),new PolyRec_Point(92,193),new PolyRec_Point(78,185),new PolyRec_Point(68,181),new PolyRec_Point(59,180),new PolyRec_Point(54,178),new PolyRec_Point(52,178),new PolyRec_Point(53,180),new PolyRec_Point(61,182),new PolyRec_Point(75,184),new PolyRec_Point(95,182),new PolyRec_Point(116,178),new PolyRec_Point(137,176),new PolyRec_Point(155,173),new PolyRec_Point(170,170),new PolyRec_Point(177,169),new PolyRec_Point(182,167),new PolyRec_Point(181,170),new PolyRec_Point(173,178),new PolyRec_Point(171,179),new PolyRec_Point(156,191),new PolyRec_Point(139,205),new PolyRec_Point(121,221),new PolyRec_Point(105,240),new PolyRec_Point(96,254),new PolyRec_Point(90,263),new PolyRec_Point(86,266),new PolyRec_Point(81,269),new PolyRec_Point(81,271)));
    // var gest13 = new PolyRec_Gesture("triangle", new Array( new PolyRec_Point(134,138),new PolyRec_Point(133,140),new PolyRec_Point(132,143),new PolyRec_Point(129,147),new PolyRec_Point(128,150),new PolyRec_Point(126,153),new PolyRec_Point(123,157),new PolyRec_Point(121,160),new PolyRec_Point(117,166),new PolyRec_Point(112,172),new PolyRec_Point(106,178),new PolyRec_Point(101,184),new PolyRec_Point(95,190),new PolyRec_Point(90,196),new PolyRec_Point(85,201),new PolyRec_Point(83,203),new PolyRec_Point(80,206),new PolyRec_Point(77,210),new PolyRec_Point(75,212),new PolyRec_Point(73,214),new PolyRec_Point(76,214),new PolyRec_Point(78,213),new PolyRec_Point(82,211),new PolyRec_Point(88,210),new PolyRec_Point(96,208),new PolyRec_Point(106,206),new PolyRec_Point(114,205),new PolyRec_Point(117,205),new PolyRec_Point(129,207),new PolyRec_Point(141,209),new PolyRec_Point(152,212),new PolyRec_Point(156,214),new PolyRec_Point(169,220),new PolyRec_Point(177,222),new PolyRec_Point(185,225),new PolyRec_Point(190,225),new PolyRec_Point(192,225),new PolyRec_Point(192,222),new PolyRec_Point(189,221),new PolyRec_Point(181,215),new PolyRec_Point(179,214),new PolyRec_Point(170,207),new PolyRec_Point(167,204),new PolyRec_Point(159,193),new PolyRec_Point(157,190),new PolyRec_Point(146,175),new PolyRec_Point(138,167),new PolyRec_Point(133,160),new PolyRec_Point(127,156),new PolyRec_Point(124,152)));
    // var gest14 = new PolyRec_Gesture("v", new Array( new PolyRec_Point(83,141),new PolyRec_Point(84,143),new PolyRec_Point(85,145),new PolyRec_Point(85,148),new PolyRec_Point(88,153),new PolyRec_Point(89,156),new PolyRec_Point(92,164),new PolyRec_Point(96,174),new PolyRec_Point(100,183),new PolyRec_Point(104,193),new PolyRec_Point(105,198),new PolyRec_Point(107,202),new PolyRec_Point(111,210),new PolyRec_Point(112,214),new PolyRec_Point(114,221),new PolyRec_Point(115,223),new PolyRec_Point(118,233),new PolyRec_Point(119,238),new PolyRec_Point(119,240),new PolyRec_Point(120,243),new PolyRec_Point(121,246),new PolyRec_Point(123,246),new PolyRec_Point(124,243),new PolyRec_Point(125,238),new PolyRec_Point(126,234),new PolyRec_Point(126,230),new PolyRec_Point(129,219),new PolyRec_Point(131,205),new PolyRec_Point(134,191),new PolyRec_Point(138,178),new PolyRec_Point(143,166),new PolyRec_Point(148,157),new PolyRec_Point(152,149),new PolyRec_Point(155,141),new PolyRec_Point(158,135),new PolyRec_Point(159,131),new PolyRec_Point(160,129)));
    // var gest15 = new PolyRec_Gesture("x", new Array( new PolyRec_Point(88,150),new PolyRec_Point(87,148),new PolyRec_Point(87,150),new PolyRec_Point(87,152),new PolyRec_Point(88,155),new PolyRec_Point(89,158),new PolyRec_Point(92,163),new PolyRec_Point(93,165),new PolyRec_Point(95,169),new PolyRec_Point(99,175),new PolyRec_Point(104,181),new PolyRec_Point(106,185),new PolyRec_Point(108,188),new PolyRec_Point(113,195),new PolyRec_Point(118,203),new PolyRec_Point(123,209),new PolyRec_Point(127,217),new PolyRec_Point(132,223),new PolyRec_Point(137,231),new PolyRec_Point(140,235),new PolyRec_Point(141,237),new PolyRec_Point(144,239),new PolyRec_Point(146,241),new PolyRec_Point(148,241),new PolyRec_Point(148,238),new PolyRec_Point(147,231),new PolyRec_Point(147,225),new PolyRec_Point(145,216),new PolyRec_Point(143,204),new PolyRec_Point(142,193),new PolyRec_Point(142,189),new PolyRec_Point(142,173),new PolyRec_Point(142,160),new PolyRec_Point(143,147),new PolyRec_Point(144,136),new PolyRec_Point(145,128),new PolyRec_Point(146,126),new PolyRec_Point(146,122),new PolyRec_Point(146,120),new PolyRec_Point(143,120),new PolyRec_Point(140,125),new PolyRec_Point(134,136),new PolyRec_Point(129,147),new PolyRec_Point(121,164),new PolyRec_Point(115,176),new PolyRec_Point(113,180),new PolyRec_Point(106,198),new PolyRec_Point(98,215),new PolyRec_Point(92,228),new PolyRec_Point(88,235),new PolyRec_Point(85,241),new PolyRec_Point(84,238),new PolyRec_Point(84,235),new PolyRec_Point(84,232)));
    
    // this.DataSet.push(PolyRec_DouglasPeucker(gest0));
    // this.DataSet.push(PolyRec_DouglasPeucker(gest1));
    // this.DataSet.push(PolyRec_DouglasPeucker(gest2));
    // this.DataSet.push(PolyRec_DouglasPeucker(gest3));
    // this.DataSet.push(PolyRec_DouglasPeucker(gest4));
    // this.DataSet.push(PolyRec_DouglasPeucker(gest5));
    // this.DataSet.push(PolyRec_DouglasPeucker(gest6));
    // this.DataSet.push(PolyRec_DouglasPeucker(gest7));
    // this.DataSet.push(PolyRec_DouglasPeucker(gest8));
    // this.DataSet.push(PolyRec_DouglasPeucker(gest9));
    // this.DataSet.push(PolyRec_DouglasPeucker(gest10));
    // this.DataSet.push(PolyRec_DouglasPeucker(gest11));
    // this.DataSet.push(PolyRec_DouglasPeucker(gest12));
    // this.DataSet.push(PolyRec_DouglasPeucker(gest13));
    // this.DataSet.push(PolyRec_DouglasPeucker(gest14));
    // this.DataSet.push(PolyRec_DouglasPeucker(gest15));
    // }

    //
	// The PolyRec Recognizer API begins here -- 3 methods: Recognize(), AddGesture(), DeleteUserGestures()
	//

    //Funzione che riconosce la gesture data in input
    this.Recognize = function(points)
    {
        const t0 = performance.now(); // start timer
        var gesture = new PolyRec_Gesture("input", points);
        var u = PolyRec_DouglasPeucker(gesture);
        const t1 = performance.now(); // intermediate timer

        if(u.Indexes == null || u.Indexes.length < 2) {
            const t2 = performance.now(); // stop timer
            // return new PolyRec_Result('Illegal', 0, 0);
            return ['No match', t1 - t0, t2 - t1, t2 - t0];
        }

        var templateName;
        var asset = Infinity;
        
        for (let i = 0; i < this.DataSet.length; i++) {
            
            var t = this.DataSet[i];

            if(t.PolyRec_Gesture.PointersNum == gesture.PointersNum)
            {
                var alignElements = PolyRec_AlignPolylines(u, t);
                var unknow = alignElements[0];
                var template = alignElements[1];
                var addedAngle = alignElements[2];
                var matches = alignElements[3];

                var penalty = 1 + addedAngle / (addedAngle + matches);
                var bestDist = PolyRec_DistanceAtBestAngle(unknow, template, template.PolyRec_Gesture.RotInv);

                var distance = penalty * bestDist;
                if(distance < asset) {
                    asset = distance;
                    templateName = this.DataSet[i].PolyRec_Gesture.Name;
                }
            }    
        }
        
        // var time = t1-t0;
        // var diff = Number(time) / 1000000000.0;

        if(templateName != null) {
            // var finalscore = ((2.0 - asset) / 2);
            const t2 = performance.now(); // stop timer
            return [templateName, t1 - t0, t2 - t1, t2 - t0];
            // return new PolyRec_Result(templateName, finalscore, diff);
        } else {
            const t2 = performance.now(); // stop timer
            return ['No match', t1 - t0, t2 - t1, t2 - t0];
            // return new PolyRec_Result('No match', 0, diff);
        }
    }
    
    //Aggiunge una nuova gesture al dataset
    this.AddGesture = function(name, points)
    {
        const t0 = performance.now(); // start timer
        var gesture = new PolyRec_Gesture(name, points);
        var polyline = PolyRec_DouglasPeucker(gesture);
        this.DataSet.push(polyline);

		var num = 0;
        var size = this.DataSet.length;
		for (var i = 0; i < size; i++) {
			if (this.DataSet[i].PolyRec_Gesture.Name == name)
				num++;
		}
        const t1 = performance.now(); // stop timer
        console.log([num, t1- t0])
		return [num, t1- t0];
    }
    
    //Cancella le gesture utente inserite
    this.DeleteUserGestures = function()
	{
		this.DataSet.length = new Array();
        return 0;
	}
}
//ritorna la polyline relativa alla gesture
function PolyRec_GetPoly(gesture, indexes) {
    return new PolyRec_Polyline(gesture, indexes);
}
//ritorna il punto nella gesture
function PolyRec_GetPoint(polyline, index)
{
    return polyline.PolyRec_Gesture.Points[polyline.Indexes[index]];
}
//Numero di vertici della gesture
function PolyRec_NumVertexes(polyline)
{
    return polyline.Indexes.length;
}
//Numero di segmenti della gesture
function PolyRec_NumLines(polyline)
{
    return polyline.Indexes.length - 1;
}
function PolyRec_DistanceAtBestAngle(u, t, rInvariant)
{
    var angle = rInvariant ? ANGLE_ROTATION_INVARIANT : ANGLE_ROTATION_SENSITIVE;    
    var a = PolyRec_Deg2Rad(-angle);
    var b = PolyRec_Deg2Rad(angle);
    var treshold = PolyRec_Deg2Rad(ANGLE_STEP);
    
    var uAngle = PolyRec_IndicativeAngle(u.PolyRec_Gesture, !rInvariant);
    var tAngle = PolyRec_IndicativeAngle(t.PolyRec_Gesture, !rInvariant);

    if(!rInvariant){
        uAngle = 0;
        tAngle = 0;
    }
    var vectorsU = PolyRec_CalculateVectors(u);
    var vectorsT = PolyRec_CalculateVectors(t);

    var alpha = (PHI * a) + (1.0 - PHI) * b;
    var beta = (1.0 - PHI) * a + (PHI * b);

    var pathA = PolyRec_DistanceAtAngle(vectorsU, vectorsT, -uAngle + alpha, -tAngle); 
    var pathB = PolyRec_DistanceAtAngle(vectorsU, vectorsT, -uAngle + beta, -tAngle);
    
    if( pathA != Infinity && pathB != Infinity) {
        while( Math.abs(b - a) > treshold) {
            if(pathA < pathB) {
                b = beta;
                beta = alpha;
                pathB = pathA;
                alpha = PHI * a + (1.0 - PHI) * b;
                pathA = PolyRec_DistanceAtAngle(vectorsU, vectorsT, -uAngle + alpha, -tAngle);
            } else {
                a = alpha;
                alpha = beta;
                pathA = pathB;
                beta = (1.0 - PHI) * a + PHI * b;
                pathB = PolyRec_DistanceAtAngle(vectorsU, vectorsT, -uAngle + beta, -tAngle);
            }
        }
        var finalDist = Math.min(pathA, pathB);
        return finalDist;
    } else {
        return Infinity;
    } 
}
/* Funzioni per la polilinea */
//Calcola il valore indicativo dell'angolo
function PolyRec_IndicativeAngle(gesture, sensitive) 
{
    var iAngle = Math.atan2(-(gesture.Centroid.Y - gesture.Points[0].Y), (gesture.Centroid.X - gesture.Points[0].X));
    iAngle = iAngle >= 0 ? iAngle : (2 * Math.PI + iAngle);
    var delta = 0.0;
    if(sensitive) {
        var baseOrientation = (Math.PI / 4.0) * ( Math.floor((iAngle + Math.PI / 8.0) / (Math.PI / 4.0)) );
        delta = baseOrientation + iAngle;
    } else {
        delta = iAngle;
    }
    return delta;
}
function PolyRec_DistanceAtAngle(v1, v2, theta1, theta2)
{
    var cost = 0; 
    for( let i = 0; i < v1.length; i++ ) {
        var diff = PolyRec_VectorDistance(v1[i], v2[i], theta1, theta2);
        cost = cost + diff;
    }
    return cost;
}
//Calcola la proporzione delle lunghezze
function PolyRec_LengthProportion(polyline, first, last, medium) 
{
    var all = PolyRec_LengthStartEnd(polyline.PolyRec_Gesture, polyline.Indexes[first], polyline.Indexes[last]);       
    var toMedian = PolyRec_LengthStartEnd(polyline.PolyRec_Gesture, polyline.Indexes[first], polyline.Indexes[medium] );
    return toMedian / all;
}
//lunghezza tra due nodi
function PolyRec_LengthStartEnd(gesture, start, end) 
{
    return gesture.Lengths[end] - gesture.Lengths[start];
}
//ritorna l'ultimo valore delle distanze
function PolyRec_LengthStartEnd(gesture)
{
    return gesture.Lengths[gesture.Lengths.length - 1];
}
//torna la distanza relativa ad un indice
function PolyRec_GetLengthByIndex(gesture, index)
{
    return gesture.Lengths[index];
}
//distanza totale gesture
function PolyRec_EndpointDistance(gesture)
{
    return PolyRec_Distance(gesture.Points[0], gesture.Points[gesture.Points.length - 1]);
}
//Calcola i vettori relativi alla gesture
function PolyRec_CalculateVectors(polyline)
{
    var vectors = new Array();
    for (let i = 0; i < PolyRec_NumLines(polyline); i++) { 
        vectors.push( new PolyRec_Vector(PolyRec_LineIntensity(polyline, i), PolyRec_LineSlope(polyline, i)) );
    }
    vectors.push( new PolyRec_Vector(PolyRec_InvisibleLineIntensity(polyline), PolyRec_InvisibleLineSlope(polyline.PolyRec_Gesture)) );  
    return vectors;    
}
//Calcola il centroide di una serie di punti
function PolyRec_CalculateCentroid(points)
{
	var centroid = new PolyRec_Point(0.0, 0.0);
	for (var i = 0; i < points.length; i++) {
		centroid.X += points[i].X;
		centroid.Y += points[i].Y;
	}
	centroid.X /= points.length;
	centroid.Y /= points.length;
	return centroid;
}
//Calcola il Box della gesture
function PolyRec_CalculateBoundingBox(points)
{
	var minX = +Infinity, 
    maxX = -Infinity, 
    minY = +Infinity, 
    maxY = -Infinity;
	for (var i = 0; i < points.length; i++) {
		minX = Math.min(points[i].X, minX);
		maxX = Math.max(points[i].X, maxX);
        minY = Math.min(points[i].Y, minY);
		maxY = Math.max(points[i].Y, maxY);
	}
	return new PolyRec_Rectangle(minX, minY, maxX - minX, maxY - minY);
}
//Calcola la diagonale
function PolyRec_Diagonal(box)
{ 
    return Math.sqrt( box.Height * box.Height + box.Width * box.Width );
}
//Ritorna la subgesture fra due punti
function PolyRec_PartOf(gesture, from, to)
{
    var sub = gesture.Points.slice(from, to);
    var subgesture = new PolyRec_Gesture(gesture.Name, sub);
    return subgesture;
}
//Calcola la distanza del punto dalla curva
function PolyRec_PointOnCurve(gesture, target)
{
    var referenceLength = PolyRec_LengthStartEnd(gesture) * target;
    var tempdist = 0;

    for (let i = 1; i < gesture.Points.length; i++) 
    {
        var current = gesture.Points[i];
        var previous = gesture.Points[i - 1]
        tempdist += PolyRec_Distance(current, previous);
        if(tempdist >= referenceLength)
        {
            return i;
        }
    }
        return gesture.Points.length - 1;
}
/**Intensità della linea */
function PolyRec_LineIntensity(polyline, lineNum)
{
    var len = polyline.Lengths[lineNum + 1] - polyline.Lengths[lineNum];
    var total = polyline.Lengths[PolyRec_NumLines(polyline)] + PolyRec_EndpointDistance(polyline.PolyRec_Gesture);
    return len / total;
}
function PolyRec_InvisibleLineIntensity(polyline)
{
    return PolyRec_EndpointDistance(polyline.PolyRec_Gesture) / ( polyline.Lengths[PolyRec_NumLines(polyline)] + PolyRec_EndpointDistance(polyline.PolyRec_Gesture) );
}
//Calcola l'angolo del segmento rispetto all'asse orizzontale
function PolyRec_LineAngle(first, last)
{
    var xDiff = last.X - first.X;
    var yDiff = first.Y - last.Y;
    var angle = Math.atan2(yDiff, xDiff); 
    return angle >= 0 ? angle : (2 * Math.PI + angle);
}
function PolyRec_InvisibleLineSlope(gesture)
{
    var first = gesture.Points[0];
    var last = gesture.Points[gesture.Points.length - 1];
    return PolyRec_LineAngle(first, last);
}
function PolyRec_LineSlope(polyline, lineNum)
{
    var first = PolyRec_GetPoint(polyline, lineNum);
    var last = PolyRec_GetPoint(polyline, lineNum + 1);
    return PolyRec_LineAngle(first, last);
}
function PolyRec_Angle(p0, p1, c, deg)
{
    var p0c = Math.sqrt(Math.pow(c.X - p0.X, 2) + Math.pow(c.Y - p0.Y, 2));
    var p1c = Math.sqrt(Math.pow(c.X - p1.X, 2) + Math.pow(c.Y - p1.Y, 2));
    var p0p1 = Math.sqrt(Math.pow(p1.X - p0.X, 2) + Math.pow(p1.Y - p0.Y, 2));
    var angle = Math.acos((p1c * p1c + p0c * p0c - p0p1 * p0p1) / (2 * p1c * p0c));
    if (deg) {
        return angle * (180/Math.PI);
    } else {
        return angle;
    }
}
function PolyRec_Mod(a, n)
{
    return a - Math.floor(a / n) * n;
}
function PolyRec_SlopeChange(polyline, index)
{
    if(index <= 0 || index >= polyline.Indexes.length - 1)
    {
        return 0;
    }
    var previous = PolyRec_LineSlope(polyline, index - 1);
    var next = PolyRec_LineSlope(polyline, index);
    var diff = previous - next;
    return PolyRec_Mod(diff + Math.PI, 2*Math.PI) - Math.PI;
}
function PolyRec_LengthAtAngle(polyline, index)
{
    return PolyRec_GetLengthByIndex(polyline.PolyRec_Gesture, polyline.Indexes[index]) / PolyRec_LengthStartEnd(polyline.PolyRec_Gesture);
}
//Calcola le lunghezze
function PolyRec_CalculateLengths(points)
{
    var distance = 0.0;
    var lengths = new Array();
    var tempPoint = null;
    lengths.push(distance);
    points.forEach( function(point) {
        if(tempPoint != null){
            distance += PolyRec_Distance(tempPoint, point);
            lengths.push(distance);
        }
        tempPoint = new PolyRec_Point(point.X, point.Y);
    });
    return lengths;
}
//Calcola le lunghezze della polilinea
function PolyRec_CalculateLengthsPoly(points, indexes)
{
    var distance = 0.0;
    var lengths = new Array();
    var tempPoint = null;
    lengths.push(distance);

    indexes.forEach( function(index) {
        var point = points[index];
        if(tempPoint != null){
            distance = distance + PolyRec_Distance(tempPoint, point);
            lengths.push(distance);
        }
        tempPoint = point;
    });
    return lengths;
}
//Distanza tra due punti
function PolyRec_Distance(p1, p2)
{
	var dx = p2.X - p1.X;
	var dy = p2.Y - p1.Y;
	return Math.sqrt(dx * dx + dy * dy);
}
//Calcola la distanza fra 2 vettori
function PolyRec_VectorDistance( v1, v2, r1, r2 ) {
    var x = PolyRec_RotationHorizontal(v1, r1) - PolyRec_RotationHorizontal(v2, r2);
    var y = PolyRec_RotationVertical(v1, r1) - PolyRec_RotationVertical(v2, r2);
    var distance = Math.sqrt( x * x + y * y) / 2;
    return distance;
}
function PolyRec_RotationVertical( vector, rotation ) {
    return vector.Intensity * Math.sin(vector.PolyRec_Angle + rotation);
} 
function PolyRec_RotationHorizontal( vector, rotation ) {
    return vector.Intensity * Math.cos(vector.PolyRec_Angle + rotation);
} 
//Conversione Gradi in Radianti
function PolyRec_Deg2Rad(d) 
{ 
    return (d * Math.PI / 180.0); 
}
/**Funzioni  Needleman-Wunsch due polilinee di input*/
function PolyRec_NeedlemanWunsch(input, template)
{
    var mScore = 0;
    var insertions = 0;
    var deletions = 0;
    var matches = 0;
    var alignment = new Array();
    var lengthA = PolyRec_NumLines(input);
    var lengthB = PolyRec_NumLines(template);
    var mD = new Array(lengthA + 1).fill(0).map(() => new Array(lengthB + 1).fill(0));

    //inizializza la matrice
    for (let i = 0; i <= lengthA; i++) {
        for (let j = 0; j <= lengthB; j++) {
            if (i == 0) {
                mD[i][j] = j * GAP_COST;
                // -j;
            } else if (j == 0) {
                mD[i][j] = i * GAP_COST;
                // -i;
            } else {
                mD[i][j] = 0;
            }
        }
    }
    //riempie le celle della matrice
    for (let i = 1; i <= lengthA; i++ ) {
        for(let j = 1; j <= lengthB; j++ ) {           
            var scoreDiag = mD[i - 1][j - 1] + PolyRec_Similarity(i, j, input, template);
            var scoreLeft = mD[i][j - 1] + GAP_COST;
            var scoreUp = mD[i - 1][j] + GAP_COST;
            mD[i][j] = Math.max(Math.max(scoreDiag, scoreLeft), scoreUp);
        }
    }
    //allinea le due sequenze
    var i = lengthA;
    var j = lengthB;
    mScore = mD[i][j] / (i + j);
    alignment.push(new PolyRec_Point(i,j));
    while (i > 0 && j > 0) {
        if ( mD[i][j] == mD[i - 1][j - 1] + PolyRec_Similarity(i, j, input, template) ) {
            alignment.unshift(new PolyRec_Point(i-1, j-1));      
            matches++;
            i--;
            j--;
            continue;
        } else if (mD[i][j] == mD[i][j - 1] + GAP_COST) {
            alignment.unshift(new PolyRec_Point(-1, j-1));
            insertions++;
            j--;
            continue;
        } else {
            alignment.unshift(new PolyRec_Point(i-1, -1));
            deletions++;
            i--;
            continue;
        }
    }
    //filtra i punti dell'allineamento
    var matchedPoints = new Array();
    alignment.forEach( function(point)
    {
        if(point.X != -1 && point.Y != -1) {
            matchedPoints.push(new PolyRec_Point(point.X, point.Y));
        }
    });

    return [matchedPoints, matches];
}

function PolyRec_Similarity(i, j, input, template )
{
    var weight = 1.0 - (BALANCE * Math.abs(PolyRec_LengthAtAngle(input, i - 1) - PolyRec_LengthAtAngle(template, j - 1))
                + (1 - BALANCE) * PolyRec_AngleDiff(PolyRec_SlopeChange(input, i - 1), PolyRec_SlopeChange(template, j - 1)));
    return weight;
}
//differenza tra 2 angoli
function PolyRec_AngleDiff(angleA, angleB)
{
    var diff = Math.abs(angleA - angleB);
    if (diff > Math.PI) {
        diff = 2 * Math.PI - diff;
    }
    return diff / Math.PI;
}

//funzioni PolyRec_Polyline Aligner
function PolyRec_AlignPolylines(input, template)
{
    var inputvert = new Array();
    var templatevert = new Array();

    var matchelement = PolyRec_NeedlemanWunsch(input, template);
    
    var matched = matchelement[0];
    var matches = matchelement[1] + 2;
    
    var previousX = 0;
    var previousY = 0;
    var addedI = 0;
    var addedT = 0;

    for(let i = 0; i < matched.length; i++) {
        
        var current = matched[i];

        var toInsertX = current.Y - previousY - 1;
        for(let j = 0; j < toInsertX; j++) {
            var distx = PolyRec_LengthProportion(template, previousY, current.Y, previousY + j + 1);
            PolyRec_Insert(input.PolyRec_Gesture, input.Indexes, inputvert, previousX, current.X, distx);
            addedI++;
        }
        
        var toInsertY = current.X - previousX - 1;
        for(let j = 0; j < toInsertY; j++) {
            var disty = PolyRec_LengthProportion(input, previousX, current.X, previousX + j + 1);
            PolyRec_Insert(template.PolyRec_Gesture, template.Indexes, templatevert, previousY, current.Y, disty);            
            addedT++;
        }
        previousX = current.X;
        previousY = current.Y;
    }
    input.Indexes.forEach( index => inputvert.push(index));
    inputvert.sort(function(a, b) {
        return a - b;
    });

    template.Indexes.forEach( index => templatevert.push(index));
    templatevert.sort(function(a, b) {
        return a - b;
    });

    var t = new PolyRec_Polyline(input.PolyRec_Gesture, inputvert);
    var o = new PolyRec_Polyline(template.PolyRec_Gesture, templatevert);

    return [t, o, addedI + addedT, matches];
}

function PolyRec_Insert(gesture, vertexesFrom, vertexesTo, prev, next, dist)
{
    var toAdd = vertexesFrom[prev];
    var sub = PolyRec_PartOf(gesture, vertexesFrom[prev], vertexesFrom[next]);
    toAdd += PolyRec_PointOnCurve(sub, dist);
    vertexesTo.push(toAdd);
}

//Applica l'algoritmo di douglas-peucker per la riduzione di punti
function PolyRec_DouglasPeucker(gesture)
{
    var tolerance = PolyRec_Diagonal(gesture.BoundingBox) / DPR_PARAMS.SParam;
    var vertexes = new Array();
    //Riduzione con tolleranza
    var n = gesture.Points.length;
    //se la gesture ha 2 o meno punti non può essere ridotta
    if(tolerance <= 0 || n < 3 ){
        return;
    }
    var marked = new Array();
    //i vertici da tenere sono marcati con true
    for( let i = 1; i < n - 1; i++ ) {
        marked[i] = false;
    }
    marked[0] = true;
    marked[n -1] = true;
    
    PolyRec_Reduce(gesture.Points, marked, tolerance, 0, n-1);
    
    for(let i = 0; i < n; i++) {
        if(marked[i]){
            vertexes.push(i);
        }
    }

    PolyRec_Fusion2(vertexes, gesture);
    return PolyRec_GetPoly(gesture, vertexes);
}

//rimuove l'angolo più piccolo
function PolyRec_Fusion2(vertexes, gesture)
{
    var smallest = PolyRec_SmallestAngle(vertexes, gesture);
    while(smallest != -1){
        vertexes.splice(smallest, 1);
        smallest = PolyRec_SmallestAngle(vertexes, gesture);
    }
}

//calcola l'indice dell'angolo minore
function PolyRec_SmallestAngle(vertexes, gesture)
{
    if( vertexes.length <= 2 ){
        return -1;
    }
    var min = 0;
    var smallestMeasure = 360.0;
    
    var first = vertexes[0];
    var medium = vertexes[1];
    for(let i = 2; i < vertexes.length; i++ ) {
        var last = vertexes[i];
        var angle = Math.abs( 180 - PolyRec_Angle(gesture.Points[first], gesture.Points[last], gesture.Points[medium], true) );        
        if(angle < smallestMeasure) {
            min =  medium;
            smallestMeasure = angle;
        }
        first = medium;
        medium = last;
    }
    var element = smallestMeasure > SLOPE_TRESHOLD ? -1 : min;
    if(element == -1)
        return -1;
    else    
        return vertexes.indexOf(element);
}
//Riduce il numero di indici
function PolyRec_Reduce(points, marked, tolerance, first, last)
{
    if(last <= first + 1) {
        return;
    }
    var maxDistance = 0.0;
    var indexFarthest = 0;

    var firstPoint = points[first];
    var lastPoint = points[last];
    
    for (let idx = first + 1; idx < last; idx++) {
       
        var current = points[idx];
        var distance = PolyRec_OrthogonalDistance(current, firstPoint, lastPoint);
       
        if (distance > maxDistance) {
            maxDistance = distance;
            indexFarthest = idx;
        }
    }
    if (maxDistance > tolerance) {
        marked[indexFarthest] = true;
        PolyRec_Reduce(points, marked, tolerance, first, indexFarthest);
        PolyRec_Reduce(points, marked, tolerance, indexFarthest, last);
    }
}
//Calcola la distanza ortogonale
function PolyRec_OrthogonalDistance(point, lineStart, lineEnd)
{
    if(lineStart.X == lineEnd.X && lineStart.Y == lineEnd.Y){
        return PolyRec_Distance(lineStart,point);
    }
    var area = Math.abs((1.0 * lineStart.Y * lineEnd.X + 1.0 * lineEnd.Y * point.X + 1.0 * point.Y * lineStart.X
                - 1.0 * lineEnd.Y * lineStart.X - 1.0 * point.Y * lineEnd.X - 1.0 * lineStart.Y * point.X)
                / 2.0);
    var bottom = Math.hypot(lineStart.Y - lineEnd.Y, lineStart.X - lineEnd.X);
    return area / bottom * 2.0;
}