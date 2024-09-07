#ifndef RANGER_H
#define RANGER_H

/*!
 *  \ingroup   ac_Ranger Ranger
 *  \brief     Inherits from RangerFusionInterface
 *  \details
 *  This class controls the comon methos of the Rangers.\n
 *  \version   1.0
 *  \date      2019
 *  \pre       none
 *  \bug       none reported as of 2019-04-19
 *  \warning
 */


#include "rangerinterface.h"
#include <random>
#include <chrono>


//! A Ranger class.
/*!
 * \brief The Ranger class.
 * This class inherits from RangerInterface and is used to
 * control the Rangers, containing setters and getters.
 */

class Ranger: public RangerInterface
{
public:
  Ranger();

  //! a pure virtual member.
  /*!
    \sa Radar() and Laser().
  */
  virtual std::vector<double> generateData() = 0;
  virtual int getSample(void) = 0;
  virtual std::string getModel(void) = 0;
  //Essential getters for obtaining internal private variables


  //! Essential getters for obtaining internal private variables.
  /*!
    \return The variable asked for.
  */
  unsigned int getAngularResolution(void);

  //! Essential getters for obtaining internal private variables.
  /*!
    \return The variable asked for.
  */
  int getOffset(void);

  //! Essential getters for obtaining internal private variables.
  /*!
    \return The variable asked for.
  */
  unsigned int getFieldOfView(void);

  //! Essential getters for obtaining internal private variables.
  /*!
    \return The variable asked for.
  */
  double getMax(void);

  //! Essential getters for obtaining internal private variables.
  /*!
    \return The variable asked for.
  */
  double getMin(void);

  //! Essential getters for obtaining internal private variables.
  /*!
    \return The variable asked for.
  */

  //Essential setters for setting internal private variables
  //! Essential setters for setting internal private variables.
  /*!
    \para The variable to set.
  */
  bool setAngularResolution(unsigned int) =0;

  //! Essential setters for setting internal private variables.
  /*!
    \para The variable to set.
  */
  bool setOffset(int);

  //! Essential setters for setting internal private variables.
  /*!
    \para The variable to set.
  */
  bool setFieldOfView(unsigned int);
  //See rangerinterface.h for more information

protected:
  //!A private Variable
  /*!
    of Rangers Angular Resolution
  */
  unsigned int angular_resolution_;

  //!A private Variable
  /*!
    of Rangers Off Set
  */
  int off_set_;

  //!A private Variable
  /*!
    of Rangers Field of View
  */
  unsigned int field_of_view_;

  //!A private Variable
  /*!
    of Rangers Max range
  */
  double max_;

  //!A private Variable
  /*!
    of Rangers Min range
  */
  double min_;
  //!A private Variable
  /*!
    of Ranger Model
  */
  std::string model_;

  //!A private Variable
  /*!
    to test for laser
  */
  const int laser_field_;

  //!A private Variable
  /*!
    to test for radar
  */
  const int radar_field_;

  int samples_;
  std::default_random_engine generator_;
  std::normal_distribution<double> distribution_;
  unsigned seed_;




};

#endif // RANGER_H
