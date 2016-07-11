#ifndef sumi_DOMAIN_H
#define sumi_DOMAIN_H

#include <sumi/transport_fwd.h>
#include <set>

namespace sumi {

class communicator {
 public:
  class rank_callback {
   public:
    virtual void
    rank_resolved(int global_rank, int comm_rank) = 0;
  };

  virtual int
  nproc() const = 0;

  int
  my_comm_rank() const {
    return my_comm_rank_;
  }

  virtual ~communicator(){}

  /**
   * @brief comm_to_global_rank
   * In the given communicator, map a comm-specific rank
   * to the actual global rank received at launch.
   * Can return unresolved_rank if not yet known.
   * @param comm_rank
   * @return The physical rank in the global communicator.
  */
  virtual int
  comm_to_global_rank(int comm_rank) const = 0;

  virtual int
  global_to_comm_rank(int global_rank) const = 0;

  static const int unresolved_rank = -1;

  void
  register_rank_callback(rank_callback* cback){
    rank_callbacks_.insert(cback);
  }

  void
  erase_rank_callback(rank_callback* cback){
    rank_callbacks_.erase(cback);
  }

 protected:
  communicator(int comm_rank) : my_comm_rank_(comm_rank){}

  void
  rank_resolved(int global_rank, int comm_rank);

 private:
  int my_comm_rank_;

  /**
   * Domain ranks do not need immediate resolution to physical ranks
   * If there is a delay in resolution, allow callbacks to be registered
  */
  std::set<rank_callback*> rank_callbacks_;

};

class global_communicator :
  public communicator
{
 public:
  global_communicator(transport* tport);

  int nproc() const;

  int comm_to_global_rank(int comm_rank) const;

  int global_to_comm_rank(int global_rank) const;

 private:
  transport* transport_;
};

class shifted_communicator :
  public communicator
{
 public:
  shifted_communicator(communicator* dom, int left_shift) :
    communicator((dom->my_comm_rank() - left_shift + dom->nproc()) % dom->nproc()),
    dom_(dom),
    shift_(left_shift),
    nproc_(dom->nproc()) {
  }

  int
  nproc() const {
    return dom_->nproc();
  }

  int comm_to_global_rank(int comm_rank) const {
    int shifted_rank = (comm_rank + shift_) % nproc_;
    return dom_->comm_to_global_rank(shifted_rank);
  }

  int global_to_comm_rank(int global_rank) const {
    int comm_rank = dom_->global_to_comm_rank(global_rank);
    int shifted_rank = (comm_rank - shift_ + nproc_) % nproc_;
    return shifted_rank;
  }

 private:
  communicator* dom_;
  int nproc_;
  int shift_;

};

class index_communicator :
  public communicator
{
 public:
  /**
   * @brief index_domain
   * @param nproc
   * @param proc_list
   */
  index_communicator(int comm_rank, int nproc, int* proc_list) :
    proc_list_(proc_list), nproc_(nproc),
    communicator(comm_rank)
  {
  }

  int nproc() const {
    return nproc_;
  }

  int comm_to_global_rank(int comm_rank) const {
    return proc_list_[comm_rank];
  }

  int global_to_comm_rank(int global_rank) const;

 private:
  int* proc_list_;
  int nproc_;

};

class rotate_communicator :
  public communicator
{
 public:
  /**
   * @brief rotate_domain
   * @param nproc
   * @param shift
   * @param me
   */
  rotate_communicator(int my_global_rank, int nproc, int shift) :
    nproc_(nproc), shift_(shift),
    communicator(global_to_comm_rank(my_global_rank))
  {
  }

  int nproc() const {
    return nproc_;
  }

  int comm_to_global_rank(int comm_rank) const {
    return (comm_rank + shift_) %  nproc_;
  }

  int global_to_comm_rank(int global_rank) const {
    return (global_rank + nproc_ - shift_) % nproc_;
  }

 private:
  int nproc_;
  int shift_;

};

class subrange_communicator :
  public communicator
{
 public:
  subrange_communicator(int my_global_rank, int start, int nproc) :
    nproc_(nproc), start_(start),
    communicator(global_to_comm_rank(my_global_rank))
  {
  }

  int nproc() const {
    return nproc_;
  }

  int comm_to_global_rank(int comm_rank) const {
    return comm_rank + start_;
  }

  int global_to_comm_rank(int global_rank) const {
    return global_rank - start_;
  }

 private:
  int nproc_;
  int start_;
};

}

#endif // DOMAIN_H
