import 'package:flutter/material.dart';

Widget slideshow(Slideshow slide) {
  switch (slide) {
    case Slideshow.intakeTitle:
      return const Text("intakeTitle");
    case Slideshow.intakeDesign:
      return const Text("intakeDesign");
    case Slideshow.intakeHardyBoard:
      return const Text("intakeHardyBoard");
    case Slideshow.intakePrototype:
      return const Text("intakePrototype");
    case Slideshow.intakeManufacturing:
      return const Text("intakeManufacturing");
    case Slideshow.indexer:
      return const Text("indexer");
    case Slideshow.climbersTitle:
      return const Text("climbersTitle");
    case Slideshow.climbersBrainstorm:
      return const Text("climbersBrainstorm");
    case Slideshow.climbersDesign:
      return const Text("climbersDesign");
    case Slideshow.climbersPrototype:
      return const Text("climbersPrototype");
    case Slideshow.climbersManufacturing:
      return const Text("climbersManufacturing");
    case Slideshow.outreachTitle:
      return const Text("outreachTitle");
    case Slideshow.firstLEGOLeague:
      return const Text("firstLEGOLeague");
    case Slideshow.trunkOrTreatOne:
      return const Text("trunkOrTreatOne");
    case Slideshow.trunkOrTreatTwo:
      return const Text("trunkOrTreatTwo");

    default:
      return const Text("default");
  }
}

enum Slideshow {
  intakeTitle,
  intakeDesign,
  intakeHardyBoard,
  intakePrototype,
  intakeManufacturing,
  indexer,
  climbersTitle,
  climbersBrainstorm,
  climbersDesign,
  climbersPrototype,
  climbersManufacturing,
  outreachTitle,
  firstLEGOLeague,
  trunkOrTreatOne,
  trunkOrTreatTwo,
}
