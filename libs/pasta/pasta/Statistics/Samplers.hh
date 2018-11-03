/*
 * Copyright (C) 2018  Zhijin Li
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *
 *     * Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above
 * copyright notice, this list of conditions and the following disclaimer
 * in the documentation and/or other materials provided with the
 * distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

// ---------------------------------------------------------------------------
//
// File: Samplers.hh for pasta
//
// Created by Zhijin Li
// E-mail:   <jonathan.zj.lee@gmail.com>
//
// Started on  Thu Nov  1 23:54:07 2018 Zhijin Li
// Last update Sat Nov  3 22:30:45 2018 Zhijin Li
// ---------------------------------------------------------------------------


#ifndef PASTA_SAMPLERS_HH
# define PASTA_SAMPLERS_HH

# include "samplersbase.hh"


namespace pasta
{
  // Fwd decl.
  namespace stats
  {
    template<typename T, int Dim, typename Kernel=kern::GaussKernel<T,Dim>,
             bool cond=is_pst_kernel_v<Kernel>()> class InverseSampler;

    template<typename T, int Dim, typename Proposal=rnd::RUniform<T>,
             typename Kernel=kern::GaussKernel<T,Dim>,
             bool cond=is_pst_kernel_v<Kernel>()> class RejectSampler;

    template<typename> class AdaptRejectSampler;

    template<typename T, int Dim, typename DistrFunc,
             typename Proposal=rnd::Gaussian<T>,
             mcmc Type=mcmc::random_walk> class MHSampler;

    // Tags used for MCMC proposal type
    struct general_tag     {};
    struct random_walk_tag {};
    struct independent_tag {};

    // Tags used for indicating symmetric proposals.
    struct symmetric_tag   {};
    struct asymmetric_tag  {};

    // MCMC specific dispatchers.
    template<mcmc __t> struct mcmc_dispatch { using type = void; };
    template<bool> struct symm_dispatch { using type = void; };

    template<> struct mcmc_dispatch<mcmc::general>
    { using type = random_walk_tag; };
    template<> struct mcmc_dispatch<mcmc::random_walk>
    { using type = random_walk_tag; };
    template<> struct mcmc_dispatch<mcmc::independent>
    { using type = independent_tag; };

    template<> struct symm_dispatch<true>
    { using type = symmetric_tag; };
    template<> struct symm_dispatch<false>
    { using type = asymmetric_tag; };

    template<mcmc __t> using mcmc_dispatch_t =
      typename mcmc_dispatch<__t>::type;
    template<bool cond> using symm_dispatch_t =
      typename symm_dispatch<cond>::type;

  }

  /// @ingroup group_traits
  namespace traits
  {
    /// @ingroup group_traits
    ///
    /// @brief Type traits property for the
    /// `pasta::stats::InverseSampler<T,Dim,InvCDF,false>` class:
    /// **inverse sampler using analytical icdf.**
    ///
    template<typename T, int Dim, typename InvCDF>
    struct specs<stats::InverseSampler<T,Dim,InvCDF,false> >
    {
      static const pst_id_list pst_id = pst_id_list::PASTA_SAMPLER;
      static constexpr int dim =                             Dim;
      typedef T                                          scalr_t;
      typedef InvCDF                                     cache_t;
      typedef point_dispatch_t<T,Dim>                    sampl_t;
    };

    /// @ingroup group_traits
    ///
    /// @brief Type traits property for the
    /// `pasta::stats::InverseSampler<T,Dim,Kernel,true>` class:
    /// **inverse sampler using empirical data points.**
    ///
    template<typename T, int Dim, typename Kernel>
    struct specs<stats::InverseSampler<T,Dim,Kernel,true> >
    {
      static const pst_id_list pst_id = pst_id_list::PASTA_SAMPLER;
      static constexpr int dim =                             Dim;
      typedef T                                          scalr_t;
      typedef Kernel                                     kernl_t;
      typedef typename traits::specs<Kernel>::value_t    value_t;
      typedef stats::EmpiricalStats<T,dim,value_t,1>     cache_t;
      typedef point_dispatch_t<T,Dim>                    sampl_t;
    };

    /// @ingroup group_traits
    ///
    /// @brief Type traits property for the
    /// `pasta::stats::RejectSampler<T,Dim,DistrFunc,Proposal,false>`
    /// class: **rejection sampler using analytical pdf.**
    ///
    template<typename T, int Dim, typename DistrFunc, typename Proposal>
    struct specs<stats::RejectSampler<T,Dim,DistrFunc,Proposal,false> >
    {
      static const pst_id_list pst_id = pst_id_list::PASTA_SAMPLER;
      static constexpr int dim =                             Dim;
      typedef T                                          scalr_t;
      typedef Proposal                                   propo_t;
      typedef DistrFunc                                  distr_t;
      typedef point_dispatch_t<T,Dim>                    sampl_t;
      typedef typename traits::specs<propo_t>::value_t   value_t;
      typedef typename traits::specs<propo_t>::locat_t   locat_t;
    };

    /// @ingroup group_traits
    ///
    /// @brief Type traits property for the
    /// `pasta::stats::RejectSampler<T,Dim,Kernel,Proposal,true>`
    /// class: rejection sampler using empirical data points.**
    ///
    template<typename T, int Dim, typename Proposal, typename Kernel>
    struct specs<stats::RejectSampler<T,Dim,Proposal,Kernel,true> >
    {
      static const pst_id_list pst_id = pst_id_list::PASTA_SAMPLER;
      static constexpr int dim =                             Dim;
      typedef T                                          scalr_t;
      typedef Kernel                                     kernl_t;
      typedef typename traits::specs<Kernel>::value_t    value_t;
      typedef bandwidth_dispatch_t<value_t,Dim>          bandw_t;
      typedef Eigen::Matrix<T,Dim,Eigen::Dynamic>        cache_t;
      typedef point_dispatch_t<T,Dim>                    sampl_t;
      typedef Proposal                                   propo_t;
    };

    /// @ingroup group_traits
    ///
    /// @brief Type traits property for the
    /// `pasta::stats::AdaptRejectSampler` class.
    ///
    template<typename T> struct specs<stats::AdaptRejectSampler<T> >
    {
      static const pst_id_list pst_id = pst_id_list::PASTA_SAMPLER;
      typedef T                                          scalr_t;
      typedef Eigen::Matrix<T,Eigen::Dynamic,2>          empdf_t;
    };

    /// @ingroup group_traits
    ///
    /// @brief Type traits property for the `pasta::stats::MHSampler`
    /// class.
    ///
    template<typename T, int Dim, typename DistrFunc,
             typename Proposal, mcmc Type>
    struct specs<stats::MHSampler<T,Dim,DistrFunc,Proposal,Type> >
    {
      static const pst_id_list pst_id = pst_id_list::PASTA_SAMPLER;
      static constexpr int dim =                             Dim;
      typedef Proposal                                   propo_t;
      typedef DistrFunc                                  distr_t;
      typedef T                                          scalr_t;
      typedef double                                     value_t;
      typedef point_dispatch_t<scalr_t,dim>              sampl_t;
    };
  }

  namespace stats
  {
    /// @ingroup group_stats
    ///
    /// @brief Class for inverse sampling from empirical data points.
    ///
    /// Inverse sampling works by using inverse cdf and a real uniform
    /// random generator in (0,1). It is based on the fact that the inv
    /// cdf of an arbitrary distribution is always uniform in (0,1). It
    /// samples a probability from a uniform (0,1) distribution, then
    /// finds the value corresponding to this probability in sample space
    /// via a search on the inverse cdf curve.
    ///
    /// **Kernel smooth density estimation** is used to get an empirical
    /// estimate of the cumulative distribution function from the input
    /// data points.
    ///
    /// @param T: the scalar type used for floating-point computations.
    /// @param Dim: dimension of the sampler, the same as the dimension
    /// of empirical data.
    /// @param Kernel: type of kernel used for kernel smooth estimation.
    /// Defaults to the Gaussian Kernel. Various are available.
    ///
    template<typename T, int Dim, typename Kernel>
    class InverseSampler<T,Dim,Kernel,true>:
      public abstract::samplersbase<InverseSampler<T,Dim,Kernel,true> >
    {
    public:

      using exact_t = InverseSampler<T,Dim,Kernel,true>;
      using specs_t =            traits::specs<exact_t>;
      using scalr_t =         typename specs_t::scalr_t;
      using kernl_t =         typename specs_t::kernl_t;
      using value_t =         typename specs_t::value_t;
      using sampl_t =         typename specs_t::sampl_t;
      using cache_t =         typename specs_t::cache_t;

      using parnt_t =   abstract::samplersbase<exact_t>;
      friend parnt_t;

      static constexpr int dim = specs_t::dim;

      using parnt_t::draw;

      /// Default Ctor.
      InverseSampler() = default;

      /// @brief Ctor.
      ///
      /// Construct an `InverseSampler` with input data, and optionally
      /// with specified domain and bandwidth.
      ///
      /// @param data: the input data points. Can be an Eigen dynamic vector,
      /// row or column ordered when Dim == 1. When Dim > 1, the matrix must
      /// be column ordered, i.e. each column represent a data point.
      /// @param args: variadic arguments as follows (in the very order):
      /// 1. **domain**: the sampling domain. When absent, an empirical one
      ///    will be computed from input data, based on lower and upper data
      ///    bound, **scaled by factor 1.2**.
      /// 2. **band_width**: the band width value used for kernel density
      ///    estimation. When Dim == 1, it is a scalar, and when Dim > 1, it
      ///    is an Eigen Dim x Dim matrix. When absent, the **Silverman rule
      ///    -of-thumb** will be used.
      ///
      /// @sa pasta::stats::KernelSmooth<Kernel,Dim>.
      ///
      template<typename Data, typename ...Args,
               std::enable_if_t<is_eigen_v<Data> >* = nullptr>
      explicit InverseSampler(Data &&data, Args &&...args);

      /// @brief Bind the current sampler to new input data points, domain
      /// and/or band width.
      ///
      /// This re-estimates the empirical cdf using internal kernel smooth
      /// estimator with new data point, domain and/or new band width.
      ///
      /// @param data: the input data points. Can be Eigen dynamic vector,
      /// row or column ordered when Dim == 1. When Dim > 1, the mat must
      /// be column ordered, i.e. each column represent a data point.
      /// @param args: variadic arguments as follow (in the very order):
      /// 1. **domain**: the sampling domain. When absent, an empirical one
      ///    will be computed from input data, based on lower and upper data
      ///    bound, **scaled by factor 1.2**.
      /// 2. **band_width**: the band width value used for kernel density
      ///    estimation. When Dim == 1, it is a scalar, and when Dim > 1, it
      ///    is an Eigen Dim x Dim matrix. When absent, the **Silverman rule
      ///    -of-thumb** will be used.
      ///
      template<typename Data, typename ...Args>
      void bind(Data &&data, Args &&...args);

    private:

      /// @brief Draw a sample from an empirical distribution.
      ///
      /// First generate a real uniform r.v. in (0,1); sample from it.
      /// Then interpolate the inverse of cdf to find the correponding
      /// sample value.
      ///
      /// @return The sampled value.
      ///
      sampl_t draw_impl() const;

      /// @brief Reset the sampler state.
      ///
      /// This resets the state of the internal random variables and
      /// distributions.
      ///
      /// Together with the `pasta::utils::reset_shared_engine()`
      /// function, they can be used to reproduce the same simulation
      /// results.
      ///
      exact_t& reset_state_impl();

      KernelSmooth<kernl_t,Dim> _estimator; //!< Kernel smooth estimator.
      rnd::RUniform<scalr_t>    _generator; //!< Uniform real (0,1) sampler.
      cache_t                   _cdf_cache; //!< Kernel smooth estimated cdf.
    };

    /// @ingroup group_stats
    ///
    /// @brief Class for rejection sampling from empirical data points.
    ///
    /// Rejection sampling is based on the idea of having one **proposal**
    /// distribution @f$ q(x) @f$ and a magnification factor @f$ M @f$,
    /// such that, @f$ Mq(x) @ge p(x) @f$ with @f$ p(x) @f$ being the pdf
    /// to sample from.
    ///
    /// Basically, if one achieves to sample uniformly, under the region
    /// covered by @f$ p(x) @f$, then the samples projected onto the sample
    /// hyper-plane follow the distribution specified by @f$ p(x) @f$.
    ///
    /// Since it is not easy to sample uniformly under @f$ p(x) @f$ region,
    /// one needs to have a proposal distribution @f$ q(x) @f$ which is
    /// normally much easier to sample from. The work-flow is then:
    /// 1. One samples from @f$ q(x) @f$.
    /// 2. Sample a uniform real random value in (0,1) @f$ u @f$.
    /// 3. Compare @f$ u @f$ with @f$ \frac{p(x)}{Mq(x)} @f$. If @f$ u @f$
    /// is greater, reject the sample. Otherwise accept the sample.
    /// 4. Continue 1-3 desired times.
    ///
    /// Rejection sampler using empirical data stores the input data points
    /// in order to estimate the pdf @f$ p(x) @f$. **Kernel smooth density
    /// estimation** is used with its default setting.
    ///
    /// @warning
    /// * If no bounded domain assumption is made on the target distribution
    ///   you might consider using **a proposal density with infinite domain,
    ///   such as a Gaussian density.**
    /// * It is benchmarked that with **input data simulated from a Gaussian
    ///   distribution**, when using **Gaussian proposal with empirical
    //    rejection sampler**, if the magnification factor is not specified
    ///   in constructor. That is, if the factor m is evaluated by a grid
    ///   search, you might get occasionally **high m value**, which means
    ///   **low acceptance rate**, due to numerical instability. Therefore,
    ///   if the target distribution has a lower and upper bound, it might be
    ///   preferable to use a **uniform proposal**, since it is more stable.
    ///
    /// @param T: the scalar type used for floating-point computations.
    /// @param Dim: dimension of the sampler, the same as the dimension
    /// of empirical data.
    /// @param Proposal: the proposal distribution. Defaults to uniform with
    /// min and max estimated from the input data, with domain scaled by 1.2.
    /// If user-defined, it must offer at least the following public APIs:
    /// 1. an `operator ()` taking a sample value returning a probability.
    /// 2. a method called `draw` to sample from current state of the proposal
    ///    density.
    /// 3. a method called `reset_state` to reset the state of the distribution
    ///    if it has states; otherwise `reset_state` can be an empty function.
    /// @param Kernel: type of kernel used for kernel smooth estimation.
    /// Defaults to the Gaussian Kernel. Various are available.
    ///
    template<typename T, int Dim, typename Proposal, typename Kernel>
    class RejectSampler<T,Dim,Proposal,Kernel,true>:
      public abstract::samplersbase<RejectSampler<T,Dim,Proposal,Kernel,true> >
    {
    public:
      using exact_t = RejectSampler<T,Dim,Proposal,Kernel,true>;
      using specs_t =                    traits::specs<exact_t>;
      using scalr_t =                 typename specs_t::scalr_t;
      using kernl_t =                 typename specs_t::kernl_t;
      using value_t =                 typename specs_t::value_t;
      using bandw_t =                 typename specs_t::bandw_t;
      using propo_t =                 typename specs_t::propo_t;
      using sampl_t =                 typename specs_t::sampl_t;
      using cache_t =                 typename specs_t::cache_t;

      using parnt_t =           abstract::samplersbase<exact_t>;
      friend parnt_t;

      using parnt_t::draw;

      /// @brief Default Ctor.
      RejectSampler() = default;;

      /// @brief Ctor.
      ///
      /// @brief Initialize a rejection sampler object using input data
      /// points proposal distribution and magnification factor m.
      ///
      /// The sampler caches the input data points for latter pdf eval.
      /// Evaluation of pdf will be done using kernel smooth method with
      /// Gaussian kernel and default settings.
      ///
      /// @param data: the input data points.
      /// @param proposal: the proposal distribution, must be a pasta
      /// distribution type.
      /// @param factor_m: the maginification factor.
      ///
      template<typename Data, typename Propo,
               enable_if_all_t<is_eigen_v<Data>,
                               is_pst_distr_v<Propo>()>* = nullptr>
      RejectSampler(Data &&data, Propo &&proposal, value_t factor_m);

      /// @brief Ctor.
      ///
      /// @brief Initialize a rejection sampler object using input data
      /// points and proposal distribution.
      ///
      /// The sampler caches the input data points for latter pdf eval.
      /// Evaluation of pdf will be done using kernel smooth method with
      /// Gaussian kernel and default settings. The magnification factor
      /// will be evaluated against the proposal density on a discrete
      /// domain defined as for the kernel density estimation.
      ///
      /// @param data: the input data points.
      /// @param proposal: the proposal distribution, must be a pasta
      /// distribution type.
      ///
      template<typename Data, typename Propo,
               enable_if_all_t<is_eigen_v<Data>,
                               is_pst_distr_v<Propo>()>* = nullptr>
      RejectSampler(Data &&data, Propo &&proposal);

      /// @brief Ctor.
      ///
      /// @brief Initialize a rejection sampler object with only data pts.
      ///
      /// The sampler caches the input data points for latter pdf eval.
      /// Evaluation of pdf will be done using kernel smooth method with
      /// Gaussian kernel and default settings. The magnification factor
      /// will be evaluated against the proposal density on a discrete
      /// domain defined as for the kernel density estimation. The proposal
      /// density will be a Gaussian distribution with mean and stdev
      /// determined estimated from input data.
      ///
      /// @param data: the input data points.
      ///
      template<typename Data, std::enable_if_t<is_eigen_v<Data> >* = nullptr>
      explicit RejectSampler(Data &&data);

      /// @brief Bind current sampler to new input data pts and optionally
      /// new proposal distribution & magnification factor.
      ///
      /// This will re-evaluate the data pdf if no magnification factor
      /// is given in extra arguments.
      ///
      /// @param data: the input data pts. Same rule as for ctors.
      /// @param args: optional parameters. Maybe nothing, new proposal
      /// or with new magnification factor m.
      ///
      template<typename Data, typename ...Args>
      void bind(Data &&data,Args &&...args);

      /// @breif (Re-)Evaluate the optimal magnitude M.
      ///
      /// Kernel smooth method is used to estimate the max of data pdf from
      /// input pts. Then the estimated pdf is traversed at each locations
      /// wrt the proposal pdf to compute the maximum.
      ///
      /// @note Default bandwidth is used for KS estimate.
      ///
      /// @param dom: an input domain where the optimal ratio will be
      /// evaluated.
      /// @param scale: a scaling factor, multiplied to the evaluted max,
      /// to make sure that real maximum is covered.
      ///
      template<typename Dom>
      void eval_optimal_ratio(Dom &&dom, scalr_t scale=1.2);

    private:

      /// @brief Draw a sample from an empirical distribution.
      ///
      /// @return The sampled value.
      ///
      sampl_t draw_impl() const;

      /// @brief Reset the sampler state.
      ///
      /// This resets the state of the internal random variables and
      /// distributions.
      ///
      /// Together with the `pasta::utils::reset_shared_engine()`
      /// function, they can be used to reproduce the same simulation
      /// results.
      ///
      exact_t& reset_state_impl();

      KernelSmooth<kernl_t,Dim> _estimatr; //!< Kernel smooth estimator.
      rnd::RUniform<scalr_t>    _unif_gen; //!< Uniform real (0,1) sampler.
      cache_t                   _data_pts; //!< Cache input data for pdf estim.
      propo_t                   _proposal; //!< The proposal distribution.
      value_t                   _factor_m; //!< The magnification factor m.
    };


    /// @ingroup group_stats
    ///
    /// @brief Class for rejection sampling from an analytical distribution.
    ///
    /// Rejection sampling is based on the idea of having one **proposal**
    /// distribution @f$ q(x) @f$ and a magnification factor @f$ M @f$,
    /// such that, @f$ Mq(x) \ge p(x) @f$ with @f$ p(x) @f$ being the pdf
    /// to sample from.
    ///
    /// Basically, if one achieves to sample uniformly, under the region
    /// covered by @f$ p(x) @f$, then the samples projected onto the sample
    /// hyper-plane follow the distribution specified by @f$ p(x) @f$.
    ///
    /// Since it is not easy to sample uniformly under @f$ p(x) @f$ region,
    /// one needs to have a proposal distribution @f$ q(x) @f$ which is
    /// normally much easier to sample from. The work-flow is then:
    /// 1. One samples from @f$ q(x) @f$.
    /// 2. Sample a uniform real random value in (0,1) @f$ u @f$.
    /// 3. Compare @f$ u @f$ with @f$ \frac{p(x)}{Mq(x)} @f$. If @f$ u @f$
    /// is greater, reject the sample. Otherwise accept the sample.
    /// 4. Continue 1-3 desired times.
    ///
    /// Rejection sampler using empirical data stores the input data points
    /// in order to estimate the pdf @f$ p(x) @f$. **Kernel smooth density
    /// estimation** is used with its default setting.
    ///
    /// @warning
    /// * **No default proposal density is provided for this class**, due to
    ///   implementation technicallity. Therefore a proposal must be provided.
    /// * The domain bound for target distribution is also not assumed in
    ///   advance, so there's **no way to evaluated the optimal the factor of
    ///   magnification m**. Therefore the **factor m must also be given in
    ///   constructor**.
    ///
    /// @param T: the scalar type used for floating-point computations.
    /// @param Dim: dimension of the sampler, the same as the dimension
    /// of the proposal density.
    /// @param DistrFunc: the distribution function. Must be a functor or a
    /// lambda offering `operator()` returning the distribution value at an
    /// arbitrary position.
    /// @param Proposal: the proposal distribution. No default arg is available
    /// due to implementation detail. It can be various distribution types in
    /// `pasta` or user-customized. In case of user-customized, it must offer
    /// at least the following public APIs:
    /// 1. an `operator ()` taking a sample value returning a probability.
    /// 2. a method called `draw` to sample from current state of the proposal
    ///    density.
    /// 3. a method called `reset_state` to reset the state of the distribution
    ///    if it has states; otherwise `reset_state` can be an empty function.
    ///
    template<typename T, int Dim, typename DistrFunc, typename Proposal>
    class RejectSampler<T,Dim,DistrFunc,Proposal,false>:
      public abstract::samplersbase<RejectSampler<T,Dim,DistrFunc,Proposal,false> >
    {
    public:
      using exact_t = RejectSampler<T,Dim,DistrFunc,Proposal,false>;
      using specs_t =                        traits::specs<exact_t>;
      using scalr_t =                     typename specs_t::scalr_t;
      using propo_t =                     typename specs_t::propo_t;
      using distr_t =                     typename specs_t::distr_t;
      using sampl_t =                     typename specs_t::sampl_t;
      using value_t =                     typename specs_t::value_t;

      using parnt_t =               abstract::samplersbase<exact_t>;
      friend parnt_t;

      using parnt_t::draw;

      /// @brief Default Ctor.
      RejectSampler() = default;;

      /// @brief Ctor.
      ///
      /// Initialize a rejection sampler object using target distribution,
      /// proposal distribution and magnification factor m.
      ///
      /// @param distr: the target distribution.
      /// @param proposal: the proposal distribution.
      /// @param factor_m: the maginification factor.
      ///
      template<typename Distr, typename Propo,
               enable_if_all_t
               <std::is_same_v<std::decay_t<Distr>,
                               std::decay_t<DistrFunc> >,
                std::is_same_v<std::decay_t<Propo>,
                               std::decay_t<Proposal> > >* = nullptr>
      RejectSampler(Distr &&distr, Propo &&propo, value_t factor_m):
        _targ_dis( std::forward<Distr>(distr) ),
        _proposal( std::forward<Propo>(propo) ),
        _factor_m( factor_m ) {};

    private:

      /// @brief Draw a sample from an empirical distribution.
      ///
      /// @return The sampled value.
      ///
      sampl_t draw_impl() const;

      /// @brief Reset the sampler state.
      ///
      /// This resets the state of the internal random variables and
      /// distributions.
      ///
      /// Together with the `pasta::utils::reset_shared_engine()`
      /// function, they can be used to reproduce the same simulation
      /// results.
      ///
      exact_t& reset_state_impl();

      rnd::RUniform<scalr_t>    _unif_gen; //!< Uniform real (0,1) random sampler.
      distr_t                   _targ_dis; //!< The target distrbution
      propo_t                   _proposal; //!< The proposal distribution.
      value_t                   _factor_m; //!< The magnification factor m.
    };


    /// Class for adaptive rejection sampling (ARS).
    template<typename T> class AdaptRejectSampler
      :public abstract::samplersbase<AdaptRejectSampler<T> >
    {
    public:
      using exact_t =                    AdaptRejectSampler<T>;
      using scalr_t = typename traits::specs<exact_t>::scalr_t;

      using parnt_t =          abstract::samplersbase<exact_t>;
      friend parnt_t;

    private:

      rnd::RUniform<scalr_t> _gen;
    };

    /// @ingroup group_stats
    ///
    /// @brief **Markov Chain Monte Carlo** sampler using the **Metroplis-
    /// Hasting algorithm**.
    ///
    /// The MH-MCMC sampler is commonly used to sample  **analytical
    /// continuous distributions** that has no direct sampling methods.
    /// For an arbitrary distribution @f$ p(x) @f$, provided with a
    /// **proposal** density, @f$ q(x,y) @f$, the MCMC sampler works as
    /// follows:
    /// 1. It initializes a sample @f$ x^{0} @f$ from the proposal density
    ///    @f$ q(x) @f$.
    /// 2. Then for iteration @f$ n @f$, sample a candidate @f$ x^{n} @f$
    ///    from the distribution @f$ q(x^{n-1},\cdot) @f$, i.e. the
    ///    distribution **determined by the proposal using previous state**.
    ///    Obatin the candidate sample @f$ x^{n} @f$. This constructs a
    ///    typical **Markov chain**.
    /// 3. Compute the **Metropolis-Hasting ratio**:
    ///    @f$ \alpha(x^{n-1},x^{n}) =
    ///     min( \frac{ p(x^{n}) q(x^{n},x^{n-1}) }
    ///                 { p(x^{n-1}) q(x^{n-1},x^{n}) } , 1 )@f$.
    ///    Draw a uniform random value @f$ u @f$ in @f$ (0,1) @f$.
    ///    * If @f$ u < \alpha(x^{n-1},x^{n}) @f$, **accept** @f$ x^{n} @f$.
    ///    * Otherwise set @f$ x^{n} = x^{n+1} @f$.
    /// 4. Repeat step 2 - 3 @f$ M @f$ times (called the **burn-in period**)
    ///    to ensure convergence to **the stationary distribution (which is
    ///    the targeting distribution @f$ p(x) @f$ under ergodic conditions).**
    ///    Then start taking samples from the Markove chain.
    ///
    /// About the choice of **the proposal density**:
    /// * If the tag `pasta::random_walk` is specified. The proposal density
    ///   is evaluated in the **random walk fashion**. This means, the density
    ///   of generating @f$ x^{n} @f$ from @f$ x^{n+1} @f$ is
    ///   @f$ q(x^{n} - x^{n+1}) @f$. @f$ q(x) @f$ can be symmetric or not.
    ///   In this case:
    ///   * the **center of the proposal distribution is adaptive** and is
    ///     evaluated for each sample iteration. This is **generally true (see
    ///     warning section for more detail)**;
    ///   * the **spread of the proposal is thus the only important factor**.
    ///
    /// * If the tag `pasta::independent` is specified. The proposal density
    ///   is evaluated **independently of the previous value**. It means the
    ///   density of generating @f$ x^{n} @f$ from @f$ x^{n+1} @f$ is
    ///   @f$ q(x^{n+1}) @f$. In this case:
    ///   * a **reasonable center value** need to be chosen;
    ///   * the **spread is also an important factor**.
    ///
    ///   This can be **either very efficient or very inefficient** depending
    ///   on the target distribution and the choice of proposal.
    ///
    /// * If the tag `pasta::general` is specified. The proposal density is
    ///   **assumed to have an `operator()` taking two sampls value as the
    ///   input**. I.e. the proposal should have a method taking
    ///   @f$ x^{n} @f$ and @f$ x^{n+1} @f$, and compute
    ///   @f$ q(x^{n},x^{n+1}) @f$.
    ///
    /// @warning When **random walk MCMC** is selected, the proposal density
    /// @f$ q(x) @f$ passed to the constructor **generally need to**
    /// *concentrate* **at 0 (the origin)**. This means:
    /// * if it's a symmetric density, it has **mean** 0;
    /// * if it's an asymmetric density, it has **mode** at 0.
    /// The reason is that, random walk MCMC updates each proposal using the
    /// following formula:
    ///   @f$ x^{n+1} = x^{n} + z @f$, with @f$ z \sim q(x^{n+1}-x^{n}) @f$,
    ///   informally called an **update random variable**.
    /// If @f$ q(x) @f$ does not concentrate at 0, the update variable
    /// @f$ z @f$ will have a systematic shift of value @f$ E_{p}(x) @f$.
    /// **Unless (in rare cases) this shift is desired, consider to passed**
    /// @f$ q(x) @f$ **as 0 concentrated**.
    ///
    /// @note The distribution @f$ p(x) @f$ to be sampled can **be known only
    ///  upto a normalization constant**.
    ///
    /// @param DistrFunc: a functor or lambda specifying a distr function, i.e.
    /// a class providing **`operator ()` to access the distribution value of a
    /// sample at any valid location**. For example, an `operator()` taking a
    /// const reference to a `sampl_t` expression (scalar when Dim == 1 and an
    /// Eigen Dim x 1 vector when Dim > 1) and return a probability.
    ///
    /// @param Proposal: the proposal density. Defaults to a
    /// `pasta::rnd::Gaussian` with **mean 0 and variance 1**.
    /// In case of customized distribution, it **must provides the following
    /// interfaces**:
    /// 1. an `operator ()` taking one `locat_t` (or two depending the type of
    ///    proposal), and returning a probability. This is to **compute**
    ///    @f$ q(x^{n}, x^{n+1}) @f$.
    /// 2. a method called `draw` to sample from current state of the proposal
    ///    density.
    /// 3. a method called `reset_state` to reset the state of the distribution
    ///    if it has states; otherwise `reset_state` can be an empty function.
    /// 4. The above specifications are **enough for random walk & independent
    ///    MCMC**. But for **general type MCMC**, the proposal also need to have
    ///    a public interface named `set_state()`, taking **a const reference to
    ///    sampl_t**, i.e. the current Markov chain state @f$ x^{n} @f$, then
    ///    **updates the current proposal density to the conditional density**:
    ///    @f$ q(.|x^{n}) @f$. And of course the **updated conditional density
    ///    need to provide direct sampling using the** `draw` method.
    ///
    /// @param Type: type of the sampling scheme. This **determines how the
    /// proposal density will be evaluated**. Available ones are
    /// `pasta::mcmc::random_walk`, `pasta::mcmc::independent` and
    /// `pasta::mcmc::general`. Default to `pasta::mcmc::random_walk`.
    ///
    template<typename T, int Dim, typename DistrFunc,
             typename Proposal, mcmc Type> class MHSampler
    {
    public:

      using exact_t = MHSampler<T,Dim,DistrFunc,Proposal,Type>;
      using specs_t =                   traits::specs<exact_t>;
      using propo_t =                typename specs_t::propo_t;
      using distr_t =                typename specs_t::distr_t;
      using scalr_t =                typename specs_t::scalr_t;
      using value_t =                typename specs_t::value_t;
      using sampl_t =                typename specs_t::sampl_t;

      static constexpr int dim = specs_t::dim;

      /// @brief Deault ctor.
      MHSampler(): _conv_reached( false ) {};

      /// @brief Ctor.
      ///
      /// Taking the target distribution function as input.
      ///
      /// The proposal distribution defaults to
      /// @param distr: the target distribution function.
      /// @param propo: the proposal density. If not specified, it will be
      /// default constructed from the `Proposal` template parameter. This
      /// will failed of course if `Proposal` is not default constructible.
      ///
      template<typename Distr, typename Propo=Proposal,
               enable_if_all_t
               <std::is_same_v<std::decay_t<Distr>,
                               std::decay_t<DistrFunc> >,
                std::is_same_v<std::decay_t<Propo>,
                               std::decay_t<Proposal> > >* = nullptr>
      MHSampler(Distr &&distr, Propo &&propo=propo_t{}):
        _distribution( std::forward<Distr>(distr) ),
        _prop_density( std::forward<Propo>(propo) ),
        _conv_reached( false ) {};

      /// @brief Draw samples MH-MCMC samples into an Eigen structure.
      ///
      /// @param structure: an input Eigen structure / expression, where
      /// the samples will be draw into.
      /// @param init_value: the initial starting sample point. Usually
      /// can be a random draw from the proposal distribution.
      /// @param burnin_size: number of burn-in samples: i.e. samples that
      /// will be rejected before in-taking of the Markov chain.
      ///
      template<typename Struct, typename Sample,
               typename = enable_if_all_t
               <is_eigen_v<Struct>,
                std::is_same_v<std::decay_t<Sample>,sampl_t>,
                dim_dispatch_v<Struct>()==dim> >
      void draw(Struct &structure, Sample &&init_value,
                int burnin_size=3000) const;

      /// @brief Reset the Markov chain and the sampler state.
      ///
      /// Together with the `pasta::utils::reset_shared_engine()`
      /// function, they can be used to reproduce the same simulation
      /// results.
      ///
      /// This turns the `convergence_reached` variable to false,
      /// reinitializes the cached state and reset the state of the
      /// internal densities. The next draw will **redo the burn-in
      /// period**.
      ///
      exact_t& reset_state();

    private:

      /// @brief Update current state of Markov chain.
      ///
      void update_current_state() const;

      /// @brief Run `burnin_size` steps of the Markov chain in order to
      /// reach the stationary distribution.
      ///
      /// @param burnin_size: number of burnin samples: i.e. samples that
      /// will be rejected before in-taking.
      ///
      void run_burnin(int burnin_size) const;

      /// @brief Update current proposal sample and compute the Metropolis-
      /// Hasting ratio.
      ///
      /// This is the specialization for **random_walk MCMC**.
      ///
      /// @param propo_sample: a sample from proposal.
      /// @return The computed Metroplis-Hasting ratio.
      ///
      void make_proposal(random_walk_tag type_tag) const;

      /// @brief Update current proposal sample and compute the Metropolis-
      /// Hasting ratio.
      ///
      /// This is the specialization for **independent MCMC**.
      ///
      /// @param propo_sample: a sample from proposal.
      /// @return The computed Metroplis-Hasting ratio.
      ///
      void make_proposal(independent_tag type_tag) const;

      /// @brief Update current proposal sample and compute the Metropolis-
      /// Hasting ratio.
      ///
      /// This is the specialization for **general MCMC**.
      ///
      /// @param propo_sample: a sample from proposal.
      /// @return The computed Metroplis-Hasting ratio.
      ///
      void make_proposal(general_tag type_tag) const;

      ///@{
      /// @brief Compute the Metropolis-Hasting ratio using current proposal
      /// and current state
      ///
      /// @return The computed Metropolis-Hasting ratio.
      ///
      value_t mh_ratio(random_walk_tag type_tag,
                       symmetric_tag sym_tag) const;

      value_t mh_ratio(random_walk_tag type_tag,
                       asymmetric_tag asym_tag) const;

      value_t mh_ratio(general_tag) const;
      ///@}

      distr_t                _distribution; //!< Distribution to sample.
      mutable propo_t        _prop_density; //!< Proposal density.
      mutable sampl_t        _currnt_state; //!< Current chain state.
      mutable sampl_t        _currnt_propo; //!< Current proposal sample.
      rnd::RUniform<value_t> _unif_sampler; //!< A Uniform real (0,1) sampler.
      mutable bool           _conv_reached; //!< Whether convergence reached.
    };

  } //!stats
} //!pasta


# include "Samplers.hxx"
#endif
