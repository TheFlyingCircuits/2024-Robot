import 'package:flutter/material.dart';

List<String> slideImageNames = ['intakeDesignLeft', 'intakeDesignRight'];

List<Image> slideImages = [];

Widget slideshow(Slideshow slide) {
  for (String slideImageName in slideImageNames) {
    slideImages.add(Image.asset(
      'assets/images/$slideImageName.png',
      filterQuality: FilterQuality.medium,
    ));
  }

  switch (slide) {
    case Slideshow.intakeTitle:
      return _title('Intake');
    case Slideshow.intakeDesign:
      return _slideWithImages(
          'Intake Design', [slideImages[0], slideImages[1]]);
    // return const Text('intakeDesign');
    case Slideshow.intakeHardyBoard:
      return const Text('intakeHardyBoard');
    case Slideshow.intakePrototype:
      return const Text('intakePrototype');
    case Slideshow.intakeManufacturing:
      return const Text('intakeManufacturing');
    case Slideshow.indexer:
      return const Text('indexer');
    case Slideshow.climbersTitle:
      return _title('Climbers');
    case Slideshow.climbersBrainstorm:
      return const Text('climbersBrainstorm');
    case Slideshow.climbersDesign:
      return const Text('climbersDesign');
    case Slideshow.climbersPrototype:
      return const Text('climbersPrototype');
    case Slideshow.climbersManufacturing:
      return const Text('climbersManufacturing');
    case Slideshow.outreachTitle:
      return _title('Outreach');
    case Slideshow.firstLEGOLeague:
      return const Text('firstLEGOLeague');
    case Slideshow.trunkOrTreatOne:
      return const Text('trunkOrTreatOne');
    case Slideshow.trunkOrTreatTwo:
      return const Text('trunkOrTreatTwo');

    default:
      return const Text('default');
  }
}

Widget _title(String title) {
  return Center(
      child: (Text(
    title,
    style: const TextStyle(
      fontSize: 100,
      fontWeight: FontWeight.bold,
    ),
  )));
}

Widget _slideWithImages(String title, List<Image> images) {
  return Center(
      child: SizedBox(
          width: 1000,
          child: Column(
            children: [
              Text(
                title,
                style: const TextStyle(
                  fontSize: 24,
                  fontWeight: FontWeight.bold,
                ),
              ),
              Expanded(
                  child: Row(
                children: [
                  if (images.isNotEmpty) Expanded(child: images[0]),
                  if (images.length > 1) const SizedBox(width: 8),
                  if (images.length > 1) Expanded(child: images[1]),
                ],
              )),
              if (images.length > 2)
                Expanded(
                    child: Row(
                  children: [
                    if (images.length > 2) images[2],
                    if (images.length > 3) images[3],
                  ],
                ))
            ],
          )));
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
