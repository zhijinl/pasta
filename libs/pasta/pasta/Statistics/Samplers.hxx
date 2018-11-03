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
// File: Samplers.hxx for pasta
//
// Created by Zhijin Li
// E-mail:   <jonathan.zj.lee@gmail.com>
//
// Started on  Thu Nov  1 23:54:25 2018 Zhijin Li
// Last update Sat Nov  3 22:31:17 2018 Zhijin Li
// ---------------------------------------------------------------------------


namespace pasta
{
  namespace stats
  {

    // =====================================================================
    template<typename T, int Dim, typename Kernel>
    template<typename Data, typename ...Args,
             std::enable_if_t<is_eigen_v<Data> >*>
    InverseSampler<T,Dim,Kernel,true>::
    InverseSampler(Data &&data, Args &&...args):
      _cdf_cache( _estimator.estimate
                  (pasta::cdf,
                   std::forward<Data>(data),
                   std::forward<Args>(args)...) )
    {};

    // =====================================================================
    template<typename T, int Dim, typename Kernel>
    template<typename Data, typename ...Args>
    void InverseSampler<T,Dim,Kernel,true>::
    bind(Data &&data, Args &&...args)
    {
      _cdf_cache = _estimator.estimate
        (pasta::cdf,std::forward<Data>(data),std::forward<Args>(args)...);
    }

    // =====================================================================
    template<typename T, int Dim, typename Kernel>
    auto InverseSampler<T,Dim,Kernel,true>::draw_impl() const -> sampl_t
    {
      value_t __tmp = _generator.draw();
      // Extreme values outside (min_cdf,max_cdf) are dragged to edge values.
      if( __tmp <= _cdf_cache.value_at(0) ) return _cdf_cache.point_at(0);
      if( __tmp >= _cdf_cache.value_at(_cdf_cache.n_elem()-1) )
        return _cdf_cache.point_at(_cdf_cache.n_elem()-1);

      auto __indx = std::find_if
        (_cdf_cache.stats().data(),
         _cdf_cache.stats().data()+ _cdf_cache.n_elem(),
         [__tmp](value_t val){ return val >= __tmp; } )
        - _cdf_cache.stats().data();

      return _cdf_cache.point_at(__indx-1) +
        ( _cdf_cache.point_at(__indx)-_cdf_cache.point_at(__indx-1) ) *
        (__tmp-_cdf_cache.value_at(__indx-1)) /
        ( _cdf_cache.value_at(__indx) - _cdf_cache.value_at(__indx-1) );
    }

    // =====================================================================
    template<typename T, int Dim, typename Kernel>
    auto InverseSampler<T,Dim,Kernel,true>::
    reset_state_impl() -> exact_t&
    {
      _generator.reset_state();
      return *this;
    }

    // =====================================================================
    template<typename T, int Dim, typename Proposal, typename Kernel>
    template<typename Data, typename Propo,
             enable_if_all_t<is_eigen_v<Data>,is_pst_distr_v<Propo>()>*>
    RejectSampler<T,Dim,Proposal,Kernel,true>::
    RejectSampler(Data &&data, Propo &&proposal, value_t factor_m):
      _data_pts( std::forward<Data>(data) ),
      _proposal( std::forward<Propo>(proposal) ),
      _factor_m( factor_m )
    {
      // TODO: this sampler caches data --> optimize it.
      _estimatr.kernel().reset( kern::silverman_bw<value_t>(_data_pts) );
    };

    // =====================================================================
    template<typename T, int Dim, typename Proposal, typename Kernel>
    template<typename Data, typename Propo,
             enable_if_all_t<is_eigen_v<Data>,is_pst_distr_v<Propo>()>*>
    RejectSampler<T,Dim,Proposal,Kernel,true>::
    RejectSampler(Data &&data, Propo &&proposal):
      _data_pts( std::forward<Data>(data) ),
      _proposal( std::forward<Propo>(proposal) )
    {
      _estimatr.kernel().reset( kern::silverman_bw<value_t>(_data_pts) );
      auto __dom = utils::make_discrete_domain<scalr_t>
        (_data_pts, utils::make_const_pt<scalr_t,Dim>(0.1), 1.2);

      eval_optimal_ratio(__dom);
    };

    // =====================================================================
    template<typename T, int Dim, typename Proposal, typename Kernel>
    template<typename Data, std::enable_if_t<is_eigen_v<Data> >*>
    RejectSampler<T,Dim,Proposal,Kernel,true>::
    RejectSampler(Data &&data):
      _data_pts( std::forward<Data>(data) )
    {
      _estimatr.kernel().reset( kern::silverman_bw<value_t>(_data_pts) );
      auto __dom = utils::make_discrete_domain<scalr_t>
        (_data_pts, utils::make_const_pt<scalr_t,Dim>(0.1), 1.2);

      _proposal = rnd::RUniform<scalr_t>
        (utils::elem_at(__dom.bounding_box(),0),
         utils::elem_at(__dom.bounding_box(),1));

      eval_optimal_ratio(__dom); // This goes after propo init.
    };

    // =====================================================================
    template<typename T, int Dim, typename Proposal, typename Kernel>
    template<typename Dom>
    void RejectSampler<T,Dim,Proposal,Kernel,true>::
    eval_optimal_ratio(Dom &&dom, scalr_t scale)
    {
      value_t __max_ratio = 0.0;
      dom.traverse
        ([&__max_ratio, this](scalr_t __pos)
         {
           auto __ratio = _estimatr.kernel()(_data_pts,__pos)
             / utils::n_elem(_data_pts) / _proposal(__pos);
           __max_ratio = std::max(__ratio, __max_ratio);
         });

      _factor_m = __max_ratio*scale;
    }

    // =====================================================================
    template<typename T, int Dim, typename Proposal, typename Kernel>
    template<typename Data, typename ...Args>
    void RejectSampler<T,Dim,Proposal,Kernel,true>::
    bind(Data &&data, Args &&...args)
    {
      (*this) = exact_t
        (std::forward<Data>(data), std::forward<Args>(args)...);
    }

    // =====================================================================
    template<typename T, int Dim, typename Proposal, typename Kernel>
    auto RejectSampler<T,Dim,Proposal,Kernel,true>::draw_impl()
      const -> sampl_t
    {
      auto __tmp = _proposal.draw();
      auto __threshold = _estimatr.kernel()(_data_pts,__tmp) /
        utils::n_elem(_data_pts) / (_factor_m*_proposal(__tmp));

      while( _unif_gen.draw() > __threshold )
      {
        __tmp = _proposal.draw();
        __threshold = _estimatr.kernel()(_data_pts,__tmp) /
          utils::n_elem(_data_pts) / (_factor_m*_proposal(__tmp));
      }
      return __tmp;
    }

    // =====================================================================
    template<typename T, int Dim, typename Proposal, typename Kernel>
    auto RejectSampler<T,Dim,Proposal,Kernel,true>::
    reset_state_impl() -> exact_t&
    {
      _unif_gen.reset_state();
      _proposal.reset_state();
      return *this;
    }

    // =====================================================================
    template<typename T, int Dim, typename DistrFunc, typename Proposal>
    auto RejectSampler<T,Dim,DistrFunc,Proposal,false>::
    draw_impl() const -> sampl_t
    {
      auto __tmp = _proposal.draw();
      auto __threshold = _targ_dis(__tmp) / (_factor_m*_proposal(__tmp));

      while( _unif_gen.draw() > __threshold )
      {
        __tmp = _proposal.draw();
        __threshold = _targ_dis(__tmp) / (_factor_m*_proposal(__tmp));
      }
      return __tmp;
    }

    // =====================================================================
    template<typename T, int Dim, typename DistrFunc, typename Proposal>
    auto RejectSampler<T,Dim,DistrFunc,Proposal,false>::
    reset_state_impl() -> exact_t&
    {
      _unif_gen.reset_state();
      _proposal.reset_state();
      return *this;
    }

    // =====================================================================
    template<typename T, int Dim, typename DistrFunc,
             typename Proposal, mcmc Type>
    template<typename Struct, typename Sample, typename>
    void MHSampler<T,Dim,DistrFunc,Proposal,Type>::
    draw(Struct &structure, Sample &&init_value, int burnin_size) const
    {
      if( !_conv_reached )
      {
        _currnt_state = std::forward<sampl_t>(init_value);
        run_burnin(burnin_size);
      }

      for(int __n = 0; __n < utils::n_elem(structure); ++__n)
      {
        update_current_state();
        utils::elem_at(structure,__n) = std::move(_currnt_propo);
      }
    }

    // =====================================================================
    template<typename T, int Dim, typename DistrFunc,
             typename Proposal, mcmc Type>
    auto MHSampler<T,Dim,DistrFunc,Proposal,Type>::
    reset_state() -> exact_t&
    {
      _currnt_state = {};
      _currnt_propo = {};
      _conv_reached = false;
      _prop_density.reset_state();
      _unif_sampler.reset_state();
      return *this;
    }

    // =====================================================================
    template<typename T, int Dim, typename DistrFunc,
             typename Proposal, mcmc Type>
    void MHSampler<T,Dim,DistrFunc,Proposal,Type>::
    update_current_state() const
    {
      make_proposal(mcmc_dispatch_t<Type>{});
      while( _unif_sampler.draw() >
             mh_ratio(mcmc_dispatch_t<Type>{},
                      symm_dispatch_t<is_symmetric_distr_v<Proposal>()>{}))
      {
        make_proposal(mcmc_dispatch_t<Type>{});
      }
      _currnt_state = _currnt_propo;
    }

    // =====================================================================
    template<typename T, int Dim, typename DistrFunc,
             typename Proposal, mcmc Type>
    void MHSampler<T,Dim,DistrFunc,Proposal,Type>::
    run_burnin(int burnin_size) const
    {
      assert( _prop_density(_currnt_state) &&
              "the initial sample should not be in positive pdf area.");

      for(int __n = 0; __n < burnin_size; ++__n)
      {
        make_proposal(mcmc_dispatch_t<Type>{});
        if( _unif_sampler.draw() <= mh_ratio
            (mcmc_dispatch_t<Type>{},
             symm_dispatch_t<is_symmetric_distr_v<Proposal>()>{}) )
          _currnt_state = _currnt_propo;
      }
      _conv_reached = true;
    }

    // =====================================================================
    template<typename T, int Dim, typename DistrFunc,
             typename Proposal, mcmc Type>
    void MHSampler<T,Dim,DistrFunc,Proposal,Type>::
    make_proposal(random_walk_tag) const
    {
      _currnt_propo = _currnt_state + _prop_density.draw();
    }

    // =====================================================================
    template<typename T, int Dim, typename DistrFunc,
             typename Proposal, mcmc Type>
    void MHSampler<T,Dim,DistrFunc,Proposal,Type>::
    make_proposal(independent_tag) const
    {
      _currnt_propo = _prop_density.draw();
    }

    // =====================================================================
    template<typename T, int Dim, typename DistrFunc,
             typename Proposal, mcmc Type>
    void MHSampler<T,Dim,DistrFunc,Proposal,Type>::
    make_proposal(general_tag) const
    {
      assert(false && "this function has not been implemented yet ...");
    }

    // =====================================================================
    template<typename T, int Dim, typename DistrFunc,
             typename Proposal, mcmc Type>
    auto MHSampler<T,Dim,DistrFunc,Proposal,Type>::
    mh_ratio(random_walk_tag, symmetric_tag) const -> value_t
    {
      return _distribution(_currnt_propo) / _distribution(_currnt_state);
    }

    // =====================================================================
    template<typename T, int Dim, typename DistrFunc,
             typename Proposal, mcmc Type>
    auto MHSampler<T,Dim,DistrFunc,Proposal,Type>::
    mh_ratio(random_walk_tag, asymmetric_tag) const -> value_t
    {
      return (_distribution(_currnt_propo)*
              _prop_density(_currnt_state-_currnt_propo))/
        (_distribution(_currnt_state)*
         _prop_density(_currnt_propo-_currnt_state));
    }

    // =====================================================================
    template<typename T, int Dim, typename DistrFunc,
             typename Proposal, mcmc Type>
    auto MHSampler<T,Dim,DistrFunc,Proposal,Type>::
    mh_ratio(general_tag) const -> value_t
    {
      return ( _distribution(_currnt_propo)*_prop_density(_currnt_state) )/
        ( _distribution(_currnt_state)*_prop_density(_currnt_propo) );
    }

  } //!stats
} //!pasta
