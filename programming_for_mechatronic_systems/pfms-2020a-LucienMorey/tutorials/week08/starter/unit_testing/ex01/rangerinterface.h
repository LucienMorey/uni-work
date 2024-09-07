#ifndef RANGERINTERFACE_H
#define RANGERINTERFACE_H

/*!
 *  \ingroup   ac_RangerInterface RangerInterface
 *  \brief     Ranger and RangerFusion base class
 *  \details
 *  This class is the pure virtual base class for all Ranger and RangerFusion.\n
 *  \version   1.0
 *  \date      2019
 *  \pre       none
 *  \bug       none reported as of 2019-04-19
 *  \warning
 */

#include <vector>
#include <string>

// The RangerInterface is a class which specifies the minimum
// required interface for your Ranger class
//! A Ranger Interface class.
/*!
 * \brief The RangerInterface class
 * This base class is used to define all methods to be used in the Ranger class.
 * used in ranger fusion to set rangers and generate data
 */
class RangerInterface
{
public:
  RangerInterface();

    //!A pure virtual member.
    /*!
      \sa Laser() and Radar()
    */
  //Generates raw data for sensor
  virtual std::vector<double> generateData() = 0;

  //Essential getters for obtaining internal private variables
  //!A pure virtual member.
  /*!
    \sa Ranger()
  */
  virtual unsigned int getAngularResolution(void) = 0;
  //!A pure virtual member.
  /*!
    \sa Ranger()
  */
  virtual int getOffset(void) = 0;
  //!A pure virtual member.
  /*!
    \sa Ranger()
  */
  virtual unsigned int getFieldOfView(void) = 0;
  //!A pure virtual member.
  /*!
    \sa Ranger()
  */
  virtual double getMax(void) = 0;
  //!A pure virtual member.
  /*!
    \sa Ranger()
  */
  virtual double getMin(void) = 0;

  //added           //can't add???
  //virtual std::string getModel(void) = 0;

  //Essential setters for setting internal private variables
  //!A pure virtual member.
  /*!
    \sa Ranger()
  */
  virtual bool setAngularResolution(unsigned int) = 0;
  //!A pure virtual member.
  /*!
    \sa Ranger()
  */
  virtual bool setOffset(int) = 0;
  //!A pure virtual member.
  /*!
    \sa Ranger()
  */
  virtual bool setFieldOfView(unsigned int) = 0;
};

#endif // RANGERFUSIONINTERFACE_H
