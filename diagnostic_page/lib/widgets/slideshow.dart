import 'package:flutter/material.dart';

List<String> slideImageNames = [
  'intake/designLeft',
  'intake/designRight',
  'intake/prototypeLeft',
  'intake/prototypeRight',
  'intake/manufacturingLeft',
  'intake/manufacturingRight',
  'indexer/left',
  'indexer/right',
  'climber/prototypeLeft',
  'climber/prototypeRight',
  'climber/tube',
  'climber/upNoSlipHook',
  'climber/fallDownHook',
  'climber/carriage',
];

List<String> slideVideoNames = [];

List<Image> slideImages = slideImageNames
    .map((name) => Image.asset(
          'assets/images/$name.png',
          filterQuality: FilterQuality.medium,
        ))
    .toList();

Widget slideshow(Slideshow slide) {
  return Scaffold(
    body: Stack(
      children: [
        Center(
          child: _getSlideContent(slide),
        ),
      ],
    ),
  );
}

Widget _getSlideContent(Slideshow slide) {
  switch (slide) {
    case Slideshow.intakeTitle:
      return _getSlideTitle(slide);
    case Slideshow.intakeDesign:
      return _slideWithImages(
          'Intake Design', slideImages.getRange(0, 2).toList());
    case Slideshow.intakeHardyBoard:
      return _slideWithImages(
          'Intake Hardy Board', slideImages.getRange(2, 4).toList());
    case Slideshow.intakePrototype:
      return const Text('Need to add images for intake prototype');
    case Slideshow.intakeManufacturing:
      return _slideWithImages(
          'Intake Manufacturing', slideImages.getRange(4, 6).toList());
    case Slideshow.indexer:
      return _slideWithImages(
          'Claws + Indexer', slideImages.getRange(6, 8).toList());
    case Slideshow.climbersTitle:
      return _getSlideTitle(slide);
    case Slideshow.climbersBrainstorm:
      return _slideWithImages(
          'Climbers Brainstorming', slideImages.getRange(8, 10).toList());
    case Slideshow.climbersDesign:
      return _slideWithImages(
          'Climbers Design', slideImages.getRange(10, 14).toList());
    default:
      return Text('Default content for ${slide.toString()}');
  }
}

Widget _getSlideTitle(Slideshow slide) {
  String name = slide.name.substring(0, slide.name.length - 5);
  switch (slide) {
    case Slideshow.intakeTitle:
    case Slideshow.climbersTitle:
    case Slideshow.outreachTitle:
      return Text(
        name[0].toUpperCase() + name.substring(1).toLowerCase(),
        style: const TextStyle(
          fontSize: 36,
          fontWeight: FontWeight.bold,
        ),
      );
    default:
      return const SizedBox.shrink(); // No title for other slides
  }
}

Widget _slideWithImages(String header, List<Image> images) {
  return Scaffold(
    body: Stack(
      children: [
        Padding(
          padding: const EdgeInsets.fromLTRB(10, 0, 10, 0),
          child: Center(
            child: SizedBox(
              height: 475,
              child: Row(
                mainAxisAlignment: MainAxisAlignment.center,
                children: (images
                    .map(
                      (image) => Expanded(
                        child: Padding(
                          padding: const EdgeInsets.all(10),
                          child: image,
                        ),
                      ),
                    )
                    .toList()),
              ),
            ),
          ),
        ),
        Positioned(
          top: 20,
          left: 20,
          child: Text(
            header,
            style: const TextStyle(
              fontSize: 24,
              fontWeight: FontWeight.bold,
            ),
          ),
        ),
      ],
    ),
  );
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
