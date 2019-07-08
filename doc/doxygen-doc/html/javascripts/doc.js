;new Vue({
  el: '#documentation-page',
  data: {
    titles: [],
    paddingTop: 50,
    searchPage: 1,
    searchPageSize: 10,
    searchInput: '',
    searchCount: 0,
    searchLoading: true,
    searchResult: []
  },
  computed: {
    showLoadMore: function () {
     return (this.searchPage) * this.searchPageSize < this.searchCount && !this.searchLoading
    }
  },
  filters: {
    docUrl: function (url) {
      if (Config.environment == 'development') {
        return url
      } else {
        return '/' + config.sdk + '/documentation' + url
      }
    },
  },
  ready: function () {
    this.initToc()
    this.initScrollFollow()
  },
  methods: {
    initToc: function () {
      var self = this,
        titlesDom = document.querySelectorAll('.article h2')

      for (var i = 0, l = titlesDom.length; i < l; i++) {
        var item = titlesDom[i]
        this.titles.push({
          hash: '#' + item.getAttribute('id'),
          text: item.innerText,
          titleDom: item,
          active: false
        })
      }
    },
    initScrollFollow: function () {
      var self = this
      $(window).on('scroll', function () {
        for (var i = 0, length = self.titles.length; i < length; i++) {
          if (i == length - 1) {
            if ($(self.titles[i].titleDom).position().top - $(window).scrollTop() < self.paddingTop) {
              self.highlight(self.titles[i])
            }
          } else {
            if ($(self.titles[i].titleDom).position().top - $(window).scrollTop() <= self.paddingTop
              &&
              $(self.titles[i + 1].titleDom).position().top - $(window).scrollTop() > self.paddingTop) {
              self.highlight(self.titles[i])
            }
          }
        }
      })
    },
    jump: function (t, e) {
      e.preventDefault()
      this.scrollTo(t.titleDom, function () {
        window.history.pushState(null, null, t.hash)
      })
    },
    scrollTo: function (target, cb) {
      cb = cb || function() {}
      $('html, body').stop().animate({scrollTop: Math.ceil($(target).position().top - this.paddingTop)}, cb);
    },
    highlight: function (target) {
      if (!target.active) {
        for (var i = 0, length = this.titles.length; i < length; i++) {
          if (this.titles[i] == target) {
            this.titles[i].active = true
          } else {
            this.titles[i].active = false
          }
        }
      }
    },
    resetSearch: function () {
      this.searchResult = []
      this.searchPage = 1
      this.searchCount = 0
      this.searchLoading = true
    },
    search: function () {
      if (!this.searchInput) return false
      var self = this
      var sdk = Config.sdk.replace('-sdk', '')
      this.resetSearch()
      // send ga
      ga('send', 'event', sdk, 'doc-search', this.searchInput)
      AjaxManager.searchDoc({keyword: this.searchInput, page: this.searchPage, locale: Config.locale, sdk: sdk}).done(function (data) {
        self.searchResult = data.results
        self.searchPageSize = data.page_size || 10
        self.searchCount = data.total_count
        self.searchLoading = false
        self.showSearch()
      }).fail(function () {
        self.searchLoading = false
        alert('Server Error')
      })
    },
    loadMoreSearch: function () {
      var self = this
      var sdk = Config.sdk.replace('-sdk', '')
      this.searchPage += 1
      self.searchLoading = true
      AjaxManager.searchDoc({keyword: this.searchInput, page: this.searchPage, locale: Config.locale, sdk: sdk}).done(function (data) {
        self.searchResult = self.searchResult.concat(data.results)
        self.searchLoading = false
      }).fail(function () {
        alert('Server Error')
        self.searchLoading = false
      })
    },
    showSearch: function () {
      $('#search-modal').modal({
        show: true,
        // backdrop: 'static',
        keyboard: false
      })
    },
    closeSearch: function () {
      $('#search-modal').modal('hide')
    }
  }
})
