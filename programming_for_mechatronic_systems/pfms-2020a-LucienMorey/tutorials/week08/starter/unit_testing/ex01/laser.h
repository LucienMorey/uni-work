#ifndef LASER_H
#define LASER_H

/*!
 *  \ingroup   ac_Laser
 *  \brief     Inherits from Ranger
 *  \details
 *  This class is the virtual sensor class.\n
 *  \version   1.0
 *  \date      2019
 *  \pre       none
 *  \bug       none reported as of 2019-04-19
 *  \warning
 */

#include "ranger.h"

//! A Laser class.
/*!
 * \brief The Laser class.
 * This class inherits from Ranger and is used to
 * simulate the Radar sensor, generating data.
 */

class Laser: public Ranger
{
public:
  //Default constructor - should set all sensor attributes to a default value
    //! Default constructor.
    /*!
      sets all sensor attributes to a default value
    */
  Laser();

  //! Generates Raw data.
  /*!
    \return a vector of raw data in the range of sensor
  */
  std::vector<double> generateData();

  //! Gets sequence number.
  /*!
    \return the sequence number
  */
  int getSample();

  //! Gets Model.
  /*!
    \return  a string of the model type
  */
  std::string getModel();

  bool setAngularResolution(unsigned int);

  const std::vector<int> available_angular_resolutions = {10,30};

};

#endif // LASER_H
