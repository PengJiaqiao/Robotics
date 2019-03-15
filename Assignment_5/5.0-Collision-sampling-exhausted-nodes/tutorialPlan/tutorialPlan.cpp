#include <QApplication>
#include <Inventor/Qt/SoQt.h>

#include "qt_visualization/QtWindow.h"
#include "TutorialPlanSystem.h"

//Initialize the global singleton variable of the main visualization window with null.
QtWindow* QtWindow::singleton = NULL;

int
main(int argc, char** argv)
{

  // ############### Testing with GUI Start #################

  //  Create the qt application object needed for the visualization.
  // QApplication application(argc, argv);
  // QObject::connect(&application, SIGNAL(lastWindowClosed()), &application, SLOT(quit()));

  // //  The QtWindow class contains the main window needed for the visualization.
  // QtWindow* window;

  // //  Initialization of the coin 3D libraries which are needed for the visualization of the robot scenes.
  // SoQt::init(window);
  // SoDB::init();

  // //  Create the TutorialPlanSystem class which contains our roblib plan system.
  // boost::shared_ptr<TutorialPlanSystem> system(new TutorialPlanSystem());

  // //  Create our main visualization window and pass our TutorialPlanSystem to the constructor.
  // window = QtWindow::instance(system.get());

  // //  Show the main visualization window to the user.
  // window->show();

  // Run the qt application.
  // return application.exec();

  // ############### Testing with GUI end #################

  // ############### Testing without GUI Start ############

  // Specify how often Planning Algorithm should run
  int number_of_rounds = 20;


  TutorialPlanSystem system = TutorialPlanSystem();
  std::cout << "Run Planning Algorithm " << number_of_rounds << " times" << std::endl;

  for(int i = 0; i < number_of_rounds; i++) {
    std::cout << "Round Number " << i + 1 << std::endl;;
    rl::plan::VectorList path;
    bool solved = system.plan(path);
    system.reset();

  }

  return 0;

  // ############## Testing without GUI end ###############
  
}
