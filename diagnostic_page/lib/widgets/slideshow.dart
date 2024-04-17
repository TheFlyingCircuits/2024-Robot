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
  'climber/doubleHook',
  'climber/carriage',
  'climber/hardyBoardDoubleHook',
  'climber/assortedClimberPrototypes',
  'climber/hardyBoardHookVarient',
  // TODO: get higher quality version of climber manufactured stuff:
  'climber/manufacturedIsolated',
  'climber/manufacturedFullAssembly',
  'outreach/first_lego_league/one',
  'outreach/first_lego_league/two',
  'outreach/first_lego_league/three',
  'outreach/first_lego_league/four',
  'outreach/first_lego_league/five',
  'outreach/first_lego_league/six',
  'outreach/first_lego_league/seven',
  'outreach/first_lego_league/eight',
  'outreach/trunk_or_treat/one',
  'outreach/trunk_or_treat/two',
  'outreach/trunk_or_treat/three',
  'outreach/trunk_or_treat/four',
  'outreach/trunk_or_treat/five',
  'outreach/trunk_or_treat/six',
];

List<Image> slideImages = slideImageNames
    .map(
      (name) => Image.asset(
        'assets/images/$name.png',
        filterQuality: FilterQuality.medium,
      ),
    )
    .toList();

List<String> slideVideoNames = [
  'videos/intake/slowmoTest',
  'videos/intake/rapidFire',
];

List<String> slideTitles = [
  'Intake',
  'Intake Design',
  'Intake Hardy Board',
  'Intake Prototype',
  'Intake Manufacturing',
  'Claws + Indexer',
  'Climbers',
  'Climbers Brainstorming',
  'Climbers Design',
  'Climbers Prototyping',
  'Climbers Manufacturing',
  'Outreach',
  'First LEGO League',
  'Trunk Or Treat',
  'Trunk Or Treat',
];

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
    case Slideshow.climbersTitle:
    case Slideshow.outreachTitle:
      return _getSlideTitle(slide);
    case Slideshow.intakeDesign:
      return _slideWithImages(
          slideTitles[1], slideImages.getRange(0, 2).toList());
    case Slideshow.intakeHardyBoard:
      return _slideWithImages(
          slideTitles[2], slideImages.getRange(2, 4).toList());
    // case Slideshow.intakePrototype:
    // return _slideWithVideos(
    //     slideTitles[3], slideVideoNames.getRange(0, 2).toList());
    // return const Text('Need to add images for intake prototype');
    case Slideshow.intakeManufacturing:
      return _slideWithImages(
          slideTitles[4], slideImages.getRange(4, 6).toList());
    case Slideshow.indexer:
      return _slideWithImages(
          slideTitles[5], slideImages.getRange(6, 8).toList());
    case Slideshow.climbersBrainstorm:
      return _slideWithImages(
          slideTitles[7], slideImages.getRange(8, 10).toList());
    case Slideshow.climbersDesign:
      return _slideWithImages(
          slideTitles[8], slideImages.getRange(10, 15).toList());
    case Slideshow.climbersPrototype:
      return _slideWithImages(
          slideTitles[9], slideImages.getRange(15, 18).toList());
    case Slideshow.climbersManufacturing:
      return _slideWithImages(
          slideTitles[11], slideImages.getRange(18, 20).toList());
    case Slideshow.firstLEGOLeague:
      return _slideWithImagesGrid(
          slideTitles[12], slideImages.getRange(20, 28).toList());
    case Slideshow.trunkOrTreatOne:
      return _slideWithImages(
          slideTitles[13], slideImages.getRange(28, 31).toList());
    case Slideshow.trunkOrTreatTwo:
      return _slideWithImages(
          slideTitles[14], slideImages.getRange(31, 34).toList());
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
          fontSize: 100,
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
          padding: const EdgeInsets.fromLTRB(10, 10, 10, 0),
          child: Center(
            child: SizedBox(
              // height: 475,
              height: 575,

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
              fontSize: 48,
              fontWeight: FontWeight.bold,
            ),
          ),
        ),
      ],
    ),
  );
}

Widget _slideWithImagesGrid(String header, List<Image> images) {
  assert(images.length == 8, 'Exactly 8 images are required.');

  return Scaffold(
    body: Stack(
      children: [
        Padding(
          padding: const EdgeInsets.fromLTRB(10, 10, 10, 0),
          child: Center(
            child: SizedBox(
              height: 475,
              child: Column(
                mainAxisAlignment: MainAxisAlignment.center,
                children: [
                  Expanded(
                    child: Row(
                      children: images
                          .sublist(0, 4)
                          .map(
                            (image) => Expanded(
                              child: Padding(
                                padding: const EdgeInsets.all(10),
                                child: image,
                              ),
                            ),
                          )
                          .toList(),
                    ),
                  ),
                  Expanded(
                    child: Row(
                      children: images
                          .sublist(4, 8)
                          .map(
                            (image) => Expanded(
                              child: Padding(
                                padding: const EdgeInsets.all(10),
                                child: image,
                              ),
                            ),
                          )
                          .toList(),
                    ),
                  ),
                ],
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
              fontSize: 48,
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
  // intakePrototype,
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
